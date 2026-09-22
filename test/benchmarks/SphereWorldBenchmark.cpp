#include <reactphysics3d/reactphysics3d.h>
#include <reactphysics3d/utils/Profiler.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace reactphysics3d;

namespace {

struct Options {
    unsigned steps = 360, warmup = 120, repeats = 3, side = 8;
    std::string scene = "all";
};

// Count current events, excluding exits. Callback cost is included in update().
struct Events : EventListener {
    unsigned contacts = 0, triggers = 0;
    void onContact(const CollisionCallback::CallbackData& data) override {
        for (uint32 i = 0; i < data.getNbContactPairs(); i++) {
            contacts += data.getContactPair(i).getEventType() != CollisionCallback::ContactPair::EventType::ContactExit;
        }
    }
    void onTrigger(const OverlapCallback::CallbackData& data) override {
        for (uint32 i = 0; i < data.getNbOverlappingPairs(); i++) {
            triggers += data.getOverlappingPair(i).getEventType() != OverlapCallback::OverlapPair::EventType::OverlapExit;
        }
    }
};

struct Scene {
    PhysicsCommon common;
    Events events;
    PhysicsWorld* world;
    std::vector<RigidBody*> spheres;

    Scene(const std::string& name, unsigned side) {
        const bool dense = name == "dense";
        const bool triggers = name == "triggers";
        const bool sleeping = name == "sleeping";
        PhysicsWorld::WorldSettings settings;
        settings.worldName = "sphere_world_benchmark";
        settings.gravity = dense ? Vector3(0, decimal(-9.81), 0) : Vector3::zero();
        settings.isSleepingEnabled = sleeping;
        settings.defaultBounciness = 0;
        settings.defaultVelocitySolverNbIterations = 6;
        settings.defaultPositionSolverNbIterations = 3;
        world = common.createPhysicsWorld(settings);
        world->setEventListener(&events);
#ifdef IS_RP3D_PROFILING_ENABLED
        world->getProfiler()->removeAllDestinations();
#endif
        auto* sphere = common.createSphereShape(decimal(0.5));
        const decimal spacing = dense ? decimal(1.01) : triggers ? decimal(0.9) : decimal(3);
        for (unsigned y = 0; y < side; y++) {
            for (unsigned z = 0; z < side; z++) {
                for (unsigned x = 0; x < side; x++) {
                    Vector3 position(decimal(x) * spacing, decimal(0.55) + decimal(y) * spacing, decimal(z) * spacing);
                    if (name == "sparse") {
                        // Isolated diagonal pairs have overlapping AABBs but
                        // separated spheres, exercising narrow-phase rejection.
                        position.x = decimal(x / 2) * spacing + decimal(x % 2) * decimal(0.8);
                        position.y += decimal(x % 2) * decimal(0.8);
                    }
                    auto* body = world->createRigidBody(Transform(position, Quaternion::identity()));
                    body->addCollider(sphere, Transform::identity())->setIsTrigger(triggers);
                    body->updateMassPropertiesFromColliders();
                    // Common velocity preserves separation/overlaps without resets or teleports.
                    if (!dense) body->setLinearVelocity(Vector3(decimal(0.1), 0, 0));
                    if (sleeping && spheres.size() % 32 != 0) body->setIsSleeping(true);
                    spheres.push_back(body);
                }
            }
        }
        if (dense) {
            const decimal half = decimal(side - 1) * spacing / 2 + decimal(0.51);
            const decimal center = decimal(side - 1) * spacing / 2;
            auto box = [&](const Vector3& extent, const Vector3& position) {
                auto* body = world->createRigidBody(Transform(position, Quaternion::identity()));
                body->setType(BodyType::STATIC);
                body->addCollider(common.createBoxShape(extent), Transform::identity());
            };
            box(Vector3(half + 1, decimal(0.5), half + 1), Vector3(center, decimal(-0.5), center));
            // Four walls keep the pile dense throughout warmup and measurement.
            for (decimal sign : {decimal(-1), decimal(1)}) {
                box(Vector3(decimal(0.5), side, half + 1), Vector3(center + sign * (half + decimal(0.5)), side, center));
                box(Vector3(half + 1, side, decimal(0.5)), Vector3(center, side, center + sign * (half + decimal(0.5))));
            }
        }
    }

    ~Scene() { common.destroyPhysicsWorld(world); }
};

#ifdef IS_RP3D_PROFILING_ENABLED
// The existing profiler records inclusive times; parent and child rows overlap.
void printPhases(ProfileNodeIterator& iterator, const std::string& path,
                 const std::string& scene, unsigned repeat, unsigned steps) {
    unsigned child = 0;
    for (iterator.first(); !iterator.isEnd(); iterator.next(), child++) {
        const std::string name = path + "/" + iterator.getCurrentName();
        std::fprintf(stderr, "%s,%u,%s,%u,%.3f\n", scene.c_str(), repeat, name.c_str(),
                     iterator.getCurrentNbTotalCalls(), iterator.getCurrentTotalTime().count() * 1000 / steps);
        iterator.enterChild(child);
        printPhases(iterator, name, scene, repeat, steps);
        iterator.enterParent();
        // enterParent() selects the first sibling, not the child we just left.
        for (unsigned i = 0; i < child; i++) iterator.next();
    }
}
#endif

void run(const Options& options, const std::string& name, unsigned repeat) {
    Scene scene(name, options.side);
    const decimal dt = decimal(1) / 60;
    for (unsigned i = 0; i < options.warmup; i++) scene.world->update(dt);
#ifdef IS_RP3D_PROFILING_ENABLED
    scene.world->getProfiler()->reset();
#endif
    std::vector<double> times;
    times.reserve(options.steps);
    double contacts = 0, triggers = 0;
    for (unsigned i = 0; i < options.steps; i++) {
        scene.events.contacts = scene.events.triggers = 0;
        const auto start = std::chrono::steady_clock::now();
        scene.world->update(dt);
        const auto end = std::chrono::steady_clock::now();
        times.push_back(std::chrono::duration<double, std::micro>(end - start).count());
        contacts += scene.events.contacts;
        triggers += scene.events.triggers;
    }
    unsigned awake = 0;
    double checksum = 0;
    for (const auto* body : scene.spheres) {
        awake += !body->isSleeping();
        const auto position = body->getTransform().getPosition();
        const auto velocity = body->getLinearVelocity();
        if (!std::isfinite(position.lengthSquare()) || !std::isfinite(velocity.lengthSquare())) {
            throw std::runtime_error("Non-finite simulation state in " + name);
        }
        checksum += double(position.x) + 3 * double(position.y) + 7 * double(position.z) + velocity.lengthSquare();
    }
    std::sort(times.begin(), times.end());
    const double median = (times[(times.size() - 1) / 2] + times[times.size() / 2]) / 2;
    const size_t p95 = (times.size() * 95 + 99) / 100 - 1;
    std::printf("%s,%u,%zu,%u,%.2f,%.2f,%.3f,%.3f,%.9g\n", name.c_str(), repeat,
                scene.spheres.size(), awake, contacts / options.steps, triggers / options.steps,
                median, times[p95], checksum);
#ifdef IS_RP3D_PROFILING_ENABLED
    std::unique_ptr<ProfileNodeIterator> iterator(scene.world->getProfiler()->getIterator());
    printPhases(*iterator, "", name, repeat, options.steps);
#endif
}

unsigned number(const char* text, unsigned maximum, bool allowZero) {
    char* end = nullptr;
    const unsigned long value = std::strtoul(text, &end, 10);
    if (*text == '\0' || *text == '-' || *end != '\0' || value > maximum || (!allowZero && value == 0)) {
        throw std::runtime_error(std::string("Invalid numeric option: ") + text);
    }
    return static_cast<unsigned>(value);
}

}

int main(int argc, char** argv) {
    try {
        Options options;
        for (int i = 1; i < argc; i++) {
            const std::string arg = argv[i];
            if (arg == "--help") {
                std::puts("sphere_world_benchmark [--scene all|dense|sparse|triggers|sleeping] [--side 8] [--warmup 120] [--steps 360] [--repeats 3]");
                return 0;
            }
            if (++i == argc) throw std::runtime_error("Missing value for " + arg);
            if (arg == "--scene") options.scene = argv[i];
            else if (arg == "--side") options.side = number(argv[i], 32, false);
            else if (arg == "--warmup") options.warmup = number(argv[i], 1000000, true);
            else if (arg == "--steps") options.steps = number(argv[i], 1000000, false);
            else if (arg == "--repeats") options.repeats = number(argv[i], 1000, false);
            else throw std::runtime_error("Unknown option: " + arg);
        }
        if (options.scene != "all" && options.scene != "dense" && options.scene != "sparse" &&
            options.scene != "triggers" && options.scene != "sleeping") throw std::runtime_error("Unknown scene");
        std::puts("scene,repeat,spheres,awake_final,mean_contact_pairs,mean_trigger_pairs,median_step_us,p95_step_us,state_checksum");
#ifdef IS_RP3D_PROFILING_ENABLED
        std::fprintf(stderr, "scene,repeat,phase,calls,mean_inclusive_us_per_step\n");
#endif
        for (const char* name : {"dense", "sparse", "triggers", "sleeping"}) {
            if (options.scene != "all" && options.scene != name) continue;
            for (unsigned repeat = 0; repeat < options.repeats; repeat++) run(options, name, repeat);
        }
        return 0;
    }
    catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
