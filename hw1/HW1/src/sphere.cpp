#include "sphere.h"

#include <Eigen/Dense>

#include "cloth.h"
#include "configs.h"

namespace {
void generateVertices(std::vector<GLfloat>& vertices, std::vector<GLuint>& indices) {
  // See http://www.songho.ca/opengl/gl_sphere.html#sphere if you don't know how to create a sphere.
  vertices.reserve(8 * (sphereStack + 1) * (sphereSlice + 1));
  indices.reserve(6 * sphereStack * sphereSlice);

  float x, y, z, xy;  //  position

  float sectorStep = static_cast<float>(EIGEN_PI * 2 / sphereSlice);
  float stackStep = static_cast<float>(EIGEN_PI / sphereStack);
  float sectorAngle, stackAngle;

  for (int i = 0; i <= sphereStack; ++i) {
    stackAngle = static_cast<float>(EIGEN_PI / 2 - i * stackStep);  // [pi/2, -pi/2]
    xy = cosf(stackAngle);                                          // r * cos(u)
    z = sinf(stackAngle);                                           // r * sin(u)

    for (int j = 0; j <= sphereSlice; ++j) {
      sectorAngle = j * sectorStep;  // [0, 2pi]

      x = xy * cosf(sectorAngle);  // r * cos(u) * cos(v)
      y = xy * sinf(sectorAngle);  // r * cos(u) * sin(v)
      vertices.insert(vertices.end(), {x, y, z, x, y, z});
    }
  }

  unsigned int k1, k2;  // EBO index
  for (int i = 0; i < sphereStack; ++i) {
    k1 = i * (sphereSlice + 1);  // beginning of current sphereStack
    k2 = k1 + sphereSlice + 1;   // beginning of next sphereStack
    for (int j = 0; j < sphereSlice; ++j, ++k1, ++k2) {
      if (i != 0) {
        indices.insert(indices.end(), {k1, k2, k1 + 1});
      }
      // k1+1 => k2 => k2+1
      if (i != (sphereStack - 1)) {
        indices.insert(indices.end(), {k1 + 1, k2, k2 + 1});
      }
    }
  }
}
}  // namespace

Spheres& Spheres::initSpheres() {
  static Spheres spheres;
  return spheres;
}

void Spheres::addSphere(const Eigen::Ref<const Eigen::Vector4f>& position, float size) {
  if (sphereCount == _particles.getCapacity()) {
    _particles.resize(sphereCount * 2);
    _radius.resize(sphereCount * 2);
    offsets.allocate(8 * sphereCount * sizeof(float));
    sizes.allocate(2 * sphereCount * sizeof(float));
  }
  _radius[sphereCount] = size;
  _particles.position(sphereCount) = position;
  _particles.velocity(sphereCount).setZero();
  _particles.acceleration(sphereCount).setZero();
  _particles.mass(sphereCount) = sphereDensity * size * size * size;

  sizes.load(0, _radius.size() * sizeof(float), _radius.data());
  ++sphereCount;
}

Spheres::Spheres() : Shape(1, 1), sphereCount(0), _radius(1, 0.0f) {
  offsets.allocate(4 * sizeof(float));
  sizes.allocate(sizeof(float));

  std::vector<GLfloat> vertices;
  std::vector<GLuint> indices;
  generateVertices(vertices, indices);

  vbo.allocate_load(vertices.size() * sizeof(GLfloat), vertices.data());
  ebo.allocate_load(indices.size() * sizeof(GLuint), indices.data());

  vao.bind();
  vbo.bind();
  ebo.bind();

  vao.enable(0);
  vao.setAttributePointer(0, 3, 6, 0);
  glVertexAttribDivisor(0, 0);
  vao.enable(1);
  vao.setAttributePointer(1, 3, 6, 3);
  glVertexAttribDivisor(1, 0);
  offsets.bind();
  vao.enable(2);
  vao.setAttributePointer(2, 3, 4, 0);
  glVertexAttribDivisor(2, 1);
  sizes.bind();
  vao.enable(3);
  vao.setAttributePointer(3, 1, 1, 0);
  glVertexAttribDivisor(3, 1);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Spheres::draw() const {
  vao.bind();
  offsets.load(0, 4 * sphereCount * sizeof(GLfloat), _particles.getPositionData());
  GLsizei indexCount = static_cast<GLsizei>(ebo.size() / sizeof(GLuint));
  glDrawElementsInstanced(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr, sphereCount);
  glBindVertexArray(0);
}

void Spheres::collide(Shape* shape) { shape->collide(this); }
void Spheres::collide(Cloth* cloth) {
  constexpr float coefRestitution = 1.0f;
  // TODO: Collide with particle (Simple approach to handle softbody collision)
  //   1. Detect collision.
  //   2. If collided, update impulse directly to particles' velocity
  // Note:
  //   1. There are `sphereCount` spheres (sphereCount is 1 in the default scene).
  //   2. There are `particlesPerEdge * particlesPerEdge` particles.
  //   3. See TODOs in Cloth::computeSpringForce if you don't know how to access data.
  float eth = 0.1f;
  for (int s = 0; s < sphereCount; s++) {
    
        /*Eigen::Vector4f sphereVel = this->_particles.velocity(s);
        Eigen::Vector4f spherePos = this->_particles.position(s);
        float sphereMass = _particles.mass(s);
        float sphereRadius = _radius[s];
        for (int p = 0; p < particlesPerEdge * particlesPerEdge; p++) {
                Eigen::Vector4f particlePos = cloth->particles().position(p);
                Eigen::Vector4f particleVel = cloth->particles().velocity(p);
                float particleMass = cloth->particles().mass(p);

                Eigen::Vector3f delta = (spherePos - particlePos).head<3>();
                float distance = delta.norm();

                Eigen::Vector3f collisionNormal = delta.normalized();
                if (distance - sphereRadius < eth) {
                    Eigen::Vector3f va = particleVel.head<3>();
                    Eigen::Vector3f vb = sphereVel.head<3>();
                    Eigen::Vector3f va_prime =
                    (va * particleMass + sphereMass * vb + coefRestitution * sphereMass * (vb - va)) /
                    (particleMass + sphereMass);
                    Eigen::Vector3f vb_prime =
                    (vb * sphereMass + particleMass * va + coefRestitution * particleMass * (va - vb)) /
                    (particleMass + sphereMass);
                    cloth->particles().velocity(p).head<3>() = va_prime;
                    this->_particles.velocity(s).head<3>() = vb_prime;
                    float penetrationDepth = sphereRadius + eth - distance;
                    Eigen::Vector3f correction = penetrationDepth * collisionNormal;
                    cloth->particles().position(p).head<3>() -= correction;
                }
        }*/
        
        
        Eigen::Vector4f spherePos = _particles.position(s);
        float sphereRadius = _radius[s];
    
        for (int p = 0; p < particlesPerEdge * particlesPerEdge; p++) {
			Eigen::Vector4f particlePos = cloth->particles().position(p);
            Eigen::Vector3f delta = (spherePos - particlePos).head<3>();
            float distance = delta.norm();
            if (distance - sphereRadius < eth) {
                Eigen::Vector3f collisionNormal = delta.normalized();
                Eigen::Vector3f particleVelocity = cloth->particles().velocity(p).head<3>();
                Eigen::Vector3f newVelocity =
                    particleVelocity - (1 + coefRestitution) * particleVelocity.dot(collisionNormal) * collisionNormal;
                cloth->particles().velocity(p).head<3>() = newVelocity;
                float penetrationDepth = sphereRadius + eth - distance;
                Eigen::Vector3f correction = penetrationDepth * collisionNormal;
                 cloth->particles().position(p).head<3>() -= correction;
            }
        }
                  
  }
}


void Spheres::setVelocity(int i, const Eigen::Vector4f vel) { _particles.velocity(i) = vel; }
