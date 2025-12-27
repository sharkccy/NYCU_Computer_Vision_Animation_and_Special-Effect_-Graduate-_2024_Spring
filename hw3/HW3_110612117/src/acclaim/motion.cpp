#include "acclaim/motion.h"
#include <fstream>
#include <iostream>
#include <utility>
#include <vector>

#include "simulation/kinematics.h"
#include "util/helper.h"

namespace acclaim {
Motion::Motion() {}
Motion::Motion(const util::fs::path &amc_file, std::unique_ptr<Skeleton> &&_skeleton) noexcept
    : skeleton(std::move(_skeleton)) {
    postures.reserve(1024);
    if (!this->readAMCFile(amc_file)) {
        std::cerr << "Error in reading AMC file, this object is not initialized!" << std::endl;
        std::cerr << "You can call readAMCFile() to initialize again" << std::endl;
        postures.resize(0);
    }
}

const std::unique_ptr<Skeleton> &Motion::getSkeleton() const { return skeleton; }

Motion::Motion(const Motion &other) noexcept
    : skeleton(std::make_unique<Skeleton>(*other.skeleton)), postures(other.postures) {}

Motion::Motion(Motion &&other) noexcept : skeleton(std::move(other.skeleton)), postures(std::move(other.postures)) {}

Motion::Motion(const Motion &other, int startIdx, int endIdx) : skeleton(std::make_unique<Skeleton>(*other.skeleton)) {
    if (startIdx < 0 || startIdx >= other.postures.size() || endIdx <= startIdx || endIdx > other.postures.size()) {
        throw std::out_of_range("Invalid start or end index");
    }

    postures.assign(other.postures.begin() + startIdx, other.postures.begin() + endIdx);
}

Motion &Motion::operator=(const Motion &other) noexcept {
    if (this != &other) {
        skeleton.reset();
        skeleton = std::make_unique<Skeleton>(*other.skeleton);
        postures = other.postures;
    }
    return *this;
}

Motion &Motion::operator=(Motion &&other) noexcept {
    if (this != &other) {
        skeleton = std::move(other.skeleton);
        postures = std::move(other.postures);
    }
    return *this;
}

void Motion::remove(int begin, int end) {
    assert(end >= begin);
    postures.erase(postures.begin() + begin, postures.begin() + end);
}

void Motion::concatenate(Motion &m2) {
    for (int i = 0; i < m2.getFrameNum(); i++) postures.insert(postures.end(), m2.postures[i]);
}

Posture &Motion::getPosture(int FrameNum) { return postures[FrameNum]; }

std::vector<Posture> Motion::getPostures() { return postures; }

void Motion::setPosture(int FrameNum, const Posture &InPosture) { postures[FrameNum] = InPosture; }

int Motion::getFrameNum() const { return static_cast<int>(postures.size()); }

void Motion::forwardkinematics(int frame_idx) {
    kinematics::forwardSolver(postures[frame_idx], skeleton->getBonePointer(0));
    skeleton->setModelMatrices();
}
void Motion::transform(Eigen::Vector4d &newFacing, const Eigen::Vector4d &newPosition) {
    // **TODO**
    // Task: Transform the whole motion segment so that the root bone of the first posture(first frame)
    //       of the motion is located at newPosition, and its facing be newFacing.
    //       The whole motion segment must remain continuous.
    Eigen::Vector3d root_pos = postures[0].bone_translations[0].head<3>();
    Eigen::Vector4d root_rot = postures[0].bone_rotations[0];
    Eigen::Vector3d translationDiff = newPosition.head<3>() - root_pos;

    Eigen::Matrix3d ini_rot = util::rotateDegreeZYX(root_rot).toRotationMatrix();
    Eigen::Matrix3d new_rot = util::rotateDegreeZYX(newFacing).toRotationMatrix();

    Eigen::Vector3d ini_dir = ini_rot.col(2);
    Eigen::Vector3d new_dir = new_rot.col(2);

    double thetaY = std::atan2(new_dir[0], new_dir[2]) - std::atan2(ini_dir[0], ini_dir[2]);
    Eigen::Matrix3d rot_along_y;
    rot_along_y = Eigen::AngleAxisd(thetaY, Eigen::Vector3d::UnitY());

    for (auto &posture : postures) {
        Eigen::Vector4d current_rot = posture.bone_rotations[0];
        Eigen::Matrix3d current_rot_mat = util::rotateDegreeZYX(current_rot).toRotationMatrix();
        Eigen::Matrix3d new_rot = rot_along_y * current_rot_mat;
        posture.bone_rotations[0].head<3>() = new_rot.eulerAngles(2, 1, 0);
        posture.bone_rotations[0] = util::toDegree(posture.bone_rotations[0]);

        Eigen::Vector3d current_pos = posture.bone_translations[0].head<3>();
        Eigen::Vector3d new_pos = rot_along_y * (current_pos - root_pos) + root_pos + translationDiff;
        posture.bone_translations[0].head<3>() = new_pos;
    }
}

Motion blend(Motion bm1, Motion bm2, const std::vector<double> &weight) {
    // **TODO**
    // Task: Return a motion segment that blends bm1 and bm2.
    //       bm1: tail of m1, bm2: head of m2
    //       You can assume that m2's root position and orientation is aleady aligned with m1 before blending.
    //       In other words, m2.transform(...) will be called before m1.blending(m2, blendWeight, blendWindowSize) is
    //       called
    int numBlendedFrames = static_cast<int>(weight.size());
    Motion blendMotion(bm1);

    for (int i = 0; i < numBlendedFrames; i++) {
        Posture &p1 = bm1.getPosture(i);
        Posture &p2 = bm2.getPosture(i);
        Posture pb;

        pb.bone_rotations.resize(p1.bone_rotations.size());
        pb.bone_translations.resize(p1.bone_translations.size());
        int numBones = p1.bone_rotations.size();
        for (int j = 0; j < numBones; j++) {
            //Eigen::Quaterniond q1(util::rotateDegreeZYX(p1.bone_rotations[j]));
            Eigen::Quaternion q1 = util::EulerAngle2Quater(util::toRadian(p1.bone_rotations[j]).head<3>());
            // Eigen::Quaterniond q2(util::rotateDegreeZYX(p2.bone_rotations[j]));
            Eigen::Quaternion q2 = util::EulerAngle2Quater(util::toRadian(p2.bone_rotations[j]).head<3>());
            Eigen::Quaterniond interpolation = q1.slerp(weight[i], q2);
            Eigen::Vector4d v;
            v << util::Quater2EulerAngle(interpolation), 0;
            pb.bone_rotations[j] = util::toDegree(v);
            // std::cout << "weight:" << std::endl << weight[i] << std::endl;
            // std::cout << "p1:" << std::endl << p1.bone_rotations[j] << std::endl;
            // std::cout << "p2:" << std::endl << p2.bone_rotations[j] << std::endl;
            // std::cout << "pb:" << std::endl << pb.bone_rotations[j] << std::endl;
            /*pb.bone_rotations[j] = p1.bone_rotations[j] * (1 - weight[i]) + p2.bone_rotations[j] * weight[i];
            pb.bone_translations[j] = p1.bone_translations[j] * (1 - weight[i]) + p2.bone_translations[j] * weight[i];*/
            pb.bone_translations[j] = p1.bone_translations[j] * (1 - weight[i]) + p2.bone_translations[j] * weight[i];
        }
        blendMotion.setPosture(i, pb);
    }

    return blendMotion;
}

Motion Motion::blending(Motion &m2, const std::vector<double> &blendWeight, int blendWindowSize) {
    // do blending
    Motion bm1(*this, this->getFrameNum() - blendWindowSize, this->getFrameNum());
    Motion bm2(m2, 0, blendWindowSize);
    Motion bm = blend(bm1, bm2, blendWeight);
    return bm;
}

bool Motion::readAMCFile(const util::fs::path &file_name) {
    // Open AMC file
    std::ifstream input_stream(file_name);
    // Check if file successfully opened
    if (!input_stream) {
        std::cerr << "Failed to open " << file_name << std::endl;
        return false;
    }
    // There are (NUM_BONES_IN_ASF_FILE - 2) moving bones and 2 dummy bones (lhipjoint and rhipjoint)
    int movable_bones = skeleton->getMovableBoneNum();
    // Ignore header
    input_stream.ignore(1024, '\n');
    input_stream.ignore(1024, '\n');
    input_stream.ignore(1024, '\n');
    int frame_num;
    std::string bone_name;
    while (input_stream >> frame_num) {
        auto &&current_posture = postures.emplace_back(skeleton->getBoneNum());
        for (int i = 0; i < movable_bones; ++i) {
            input_stream >> bone_name;
            const Bone &bone = *skeleton->getBonePointer(bone_name);
            int bone_idx = bone.idx;
            Eigen::Vector4d bone_rotation = Eigen::Vector4d::Zero();
            Eigen::Vector4d bone_translation = Eigen::Vector4d::Zero();
            if (bone.doftx) {
                input_stream >> bone_translation[0];
            }
            if (bone.dofty) {
                input_stream >> bone_translation[1];
            }
            if (bone.doftz) {
                input_stream >> bone_translation[2];
            }
            if (bone.dofrx) {
                input_stream >> bone_rotation[0];
            }
            if (bone.dofry) {
                input_stream >> bone_rotation[1];
            }
            if (bone.dofrz) {
                input_stream >> bone_rotation[2];
            }
            current_posture.bone_rotations[bone_idx] = std::move(bone_rotation);
            current_posture.bone_translations[bone_idx] = std::move(bone_translation);
            if (bone_idx == 0) {
                current_posture.bone_translations[bone_idx] *= skeleton->getScale();
            }
        }
    }
    input_stream.close();
    std::cout << frame_num << " samples in " << file_name.string() << " are read" << std::endl;
    return true;
}
void Motion::render(graphics::Program *program) const { skeleton->render(program); }

}  // namespace acclaim
