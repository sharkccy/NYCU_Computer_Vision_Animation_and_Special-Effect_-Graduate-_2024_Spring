#include "integrator.h"

#include "configs.h"

void ExplicitEuler::integrate(const std::vector<Particles *> &particles, std::function<void(void)>) const {
  // TODO: Integrate velocity and acceleration
  //   1. Integrate velocity.
  //   2. Integrate acceleration.
  //   3. You should not compute position using acceleration. Since some part only update velocity. (e.g. impulse)
  // Note:
  //   1. You don't need the simulation function in explicit euler.
  //   2. You should do this first because it is very simple. Then you can chech your collision is correct or not.
  //   3. This can be done in 5 lines. (Hint: You can add / multiply all particles at once since it is a large matrix.)
    float dt = deltaTime;
    for (auto &p : particles) {
		p->velocity() += p->acceleration() * dt;
		p->position() += p->velocity() * dt;
	}
}

void ImplicitEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.
    std::vector<Eigen::Matrix4Xf> originalPositions(particles.size()), originalVelocities(particles.size());

    for (size_t i = 0; i < particles.size(); ++i) {
      originalPositions[i] = particles[i]->position();
      originalVelocities[i] = particles[i]->velocity();

      particles[i]->position() += particles[i]->velocity() * deltaTime;
      particles[i]->velocity() += particles[i]->acceleration() * deltaTime;
    }
    simulateOneStep();
    for (size_t i = 0; i < particles.size(); ++i) {
      particles[i]->position() = originalPositions[i] + particles[i]->velocity() * deltaTime;
      particles[i]->velocity() = originalVelocities[i] + particles[i]->acceleration() * deltaTime;
    }
  }


void MidpointEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.
  std::vector<Eigen::Matrix4Xf> originalPositions(particles.size()), originalVelocities(particles.size());

    for (size_t i = 0; i < particles.size(); ++i) {
    originalPositions[i] = particles[i]->position();
    originalVelocities[i] = particles[i]->velocity();

    particles[i]->position() += particles[i]->velocity() * deltaTime / 2;
    particles[i]->velocity() += particles[i]->acceleration() * deltaTime / 2;
    }
    simulateOneStep();
    for (size_t i = 0; i < particles.size(); ++i) {
    particles[i]->position() = originalPositions[i] + particles[i]->velocity() * deltaTime;
    particles[i]->velocity() = originalVelocities[i] + particles[i]->acceleration() * deltaTime;
    }
}


void RungeKuttaFourth::integrate(const std::vector<Particles *> &particles,
                                 std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Compute k1, k2, k3, k4
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.
    std::vector<Eigen::Matrix4Xf> 
        originalPositions(particles.size()), originalVelocities(particles.size()), 
        k1delPositions(particles.size()), k1delVelocities(particles.size()),
        k2delPositions(particles.size()), k2delVelocities(particles.size()),
        k3delPositions(particles.size()), k3delVelocities(particles.size()),
        k4delPositions(particles.size()), k4delVelocities(particles.size());
    for (size_t i = 0; i < particles.size(); ++i) {
        originalPositions[i] = particles[i]->position();
        originalVelocities[i] = particles[i]->velocity();

        k1delPositions[i] = particles[i]->velocity() * deltaTime;
        k1delVelocities[i] = particles[i]->acceleration() * deltaTime;
        particles[i]->position() += k1delPositions[i] / 2;
        particles[i]->velocity() += k1delVelocities[i] / 2;
    }
    simulateOneStep();
    for (size_t i = 0; i < particles.size(); ++i) {
        k2delPositions[i] = particles[i]->velocity() * deltaTime;
        k2delVelocities[i] = particles[i]->acceleration() * deltaTime;
        particles[i]->position() = originalPositions[i] + k2delPositions[i] / 2;
        particles[i]->velocity() = originalVelocities[i] + k2delVelocities[i] / 2;
    }
    simulateOneStep();
    for (size_t i = 0; i < particles.size(); ++i) {
		k3delPositions[i] = particles[i]->velocity() * deltaTime;
		k3delVelocities[i] = particles[i]->acceleration() * deltaTime;
		particles[i]->position() = originalPositions[i] + k3delPositions[i];
		particles[i]->velocity() = originalVelocities[i] + k3delVelocities[i];
	}
    simulateOneStep();
    for (size_t i = 0; i < particles.size(); ++i) {
        k4delPositions[i] = particles[i]->velocity() * deltaTime;
        k4delVelocities[i] = particles[i]->acceleration() * deltaTime;
        particles[i]->position() = originalPositions[i] + ((k1delPositions[i] + k2delPositions[i] * 2 + k3delPositions[i] * 2 + k4delPositions[i] * 1) / 6);
        particles[i]->velocity() = originalVelocities[i] + ((k1delVelocities[i] + k2delVelocities[i] * 2 + k3delVelocities[i] * 2 + k4delVelocities[i] * 1) / 6);
    }
}
