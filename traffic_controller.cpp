// This Project is created by Riley P.
// Real-Time Traffic Light Controller Simulator (C++)
// Design goals:
//  - Deterministic state machine timing (NS/EW phases + all-red safety buffers)
//  - Concurrency: controller thread runs the state machine; input thread queues pedestrian requests
// Safety rule:
//  - Pedestrian WALK is only granted during an ALL_RED interval to prevent conflicting green signals.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono;

// Enumerations for traffic light colors and controller phases (finite state machine)
enum class Light { Red, Yellow, Green };
enum class Phase {
    NS_Green,
    NS_Yellow,
    All_Red_1,
    EW_Green,
    EW_Yellow,
    All_Red_2,
    Ped_Walk
};

// Helpers to print enum values for readable console output
static const char* lightToStr(Light l) {
    switch (l) {
        case Light::Red: return "RED";
        case Light::Yellow: return "YELLOW";
        case Light::Green: return "GREEN";
    }
    return "UNKNOWN";
}

// Struct for how long light cycles will be
struct Config {
    int nsGreenSec  = 10;
    int nsYellowSec = 3;
    int ewGreenSec  = 10;
    int ewYellowSec = 3;
    int allRedSec   = 1;
    int pedWalkSec  = 6;
};

// Shared controller state
struct SharedState {
    std::mutex mtx;
    std::condition_variable cv;

    Phase phase = Phase::NS_Green;

    // Set by input thread, consumed by controller thread
    bool pedRequested = false;

    // For clean shutdown
    std::atomic<bool> running{true};

    // For printing and timestamps
    uint64_t tick = 0;
};

struct Snapshot {
    Phase phase;
    bool pedRequested;
    uint64_t tick;
};

static const char* phaseToStr(Phase p) {
    switch (p) {
        case Phase::NS_Green: return "NS_GREEN";
        case Phase::NS_Yellow: return "NS_YELLOW";
        case Phase::All_Red_1: return "ALL_RED";
        case Phase::EW_Green: return "EW_GREEN";
        case Phase::EW_Yellow: return "EW_YELLOW";
        case Phase::All_Red_2: return "ALL_RED";
        case Phase::Ped_Walk: return "PED_WALK";
    }
    return "UNKNOWN";
}

// Maps the current phase to light outputs and pedestrian WALK indicator
static void computeLights(Phase phase, Light& ns, Light& ew, bool& walkOn) {
    walkOn = false;
    switch (phase) {
        case Phase::NS_Green:
            ns = Light::Green; ew = Light::Red; break;
        case Phase::NS_Yellow:
            ns = Light::Yellow; ew = Light::Red; break;
        case Phase::EW_Green:
            ns = Light::Red; ew = Light::Green; break;
        case Phase::EW_Yellow:
            ns = Light::Red; ew = Light::Yellow; break;
        case Phase::All_Red_1:
        case Phase::All_Red_2:
            ns = Light::Red; ew = Light::Red; break;
        case Phase::Ped_Walk:
            ns = Light::Red; ew = Light::Red; walkOn = true; break;
    }
}

// Function for the duration of the light cycle phases
static int durationForPhase(const Config& cfg, Phase p) {
    switch (p) {
        case Phase::NS_Green:  return cfg.nsGreenSec;
        case Phase::NS_Yellow: return cfg.nsYellowSec;
        case Phase::All_Red_1: return cfg.allRedSec;
        case Phase::EW_Green:  return cfg.ewGreenSec;
        case Phase::EW_Yellow: return cfg.ewYellowSec;
        case Phase::All_Red_2: return cfg.allRedSec;
        case Phase::Ped_Walk:  return cfg.pedWalkSec;
    }
    return 1;
}

// Normal phase progression for the state machine (without pedestrian override)
static Phase nextNormalPhase(Phase p) {
    switch (p) {
        case Phase::NS_Green:  return Phase::NS_Yellow;
        case Phase::NS_Yellow: return Phase::All_Red_1;
        case Phase::All_Red_1: return Phase::EW_Green;
        case Phase::EW_Green:  return Phase::EW_Yellow;
        case Phase::EW_Yellow: return Phase::All_Red_2;
        case Phase::All_Red_2: return Phase::NS_Green;

        // after walk, resume cycle
        case Phase::Ped_Walk:  return Phase::NS_Green; 
    }
    return Phase::NS_Green;
}

// Print the status of the light system
static void printStatus(const Snapshot& s) {
    Light ns, ew;
    bool walkOn;
    computeLights(s.phase, ns, ew, walkOn);

    std::cout
        << "[t=" << s.tick << "s] "
        << "Phase=" << phaseToStr(s.phase)
        << " | NS=" << lightToStr(ns)
        << " | EW=" << lightToStr(ew)
        << " | WALK=" << (walkOn ? "ON" : "OFF")
        << " | PedReq=" << (s.pedRequested ? "YES" : "NO")
        << "\n";
}

// Controller thread of the program
static void controllerThread(SharedState& st, const Config& cfg) {
    auto start = steady_clock::now();

    while (st.running.load()) {
        // Snapshot for printing
        Snapshot snap;
        {
            std::lock_guard<std::mutex> lk(st.mtx);
            snap.phase = st.phase;
            snap.pedRequested = st.pedRequested;
            // Compute elapsed seconds as a "tick"
            auto now = steady_clock::now();
            st.tick = (uint64_t)duration_cast<seconds>(now - start).count();
            snap.tick = st.tick;
        }
        printStatus(snap);

        // Determine how long to hold current phase
        int holdSec = durationForPhase(cfg, snap.phase);

        // Sleep in 1s increments so we can stop quickly if needed
        for (int i = 0; i < holdSec; ++i) {
            if (!st.running.load()) return;
            std::this_thread::sleep_for(1s);
        }

        // Safety-critical transition rule:
        // Only service a queued pedestrian request during an ALL_RED interval.
        // This prevents WALK from activating while any direction could be green/yellow.
        {
            std::lock_guard<std::mutex> lk(st.mtx);

            bool atAllRed = (st.phase == Phase::All_Red_1 || st.phase == Phase::All_Red_2);

            if (atAllRed && st.pedRequested) {
                // Consume request and enter WALK phase
                st.pedRequested = false;
                st.phase = Phase::Ped_Walk;
            } else {
                // Normal progression
                st.phase = nextNormalPhase(st.phase);
            }
        }
    }
}

// Input thread for the program
static void inputThread(SharedState& st) {
    std::cout << "\nControls:\n"
              << "  p + Enter : request pedestrian WALK\n"
              << "  q + Enter : quit\n\n";

    std::string line;
    while (st.running.load() && std::getline(std::cin, line)) {
        if (line == "q" || line == "quit" || line == "exit") {
            st.running.store(false);
            break;
        }
        if (line == "p") {
            std::lock_guard<std::mutex> lk(st.mtx);
            st.pedRequested = true;
            std::cout << "[input] Pedestrian request queued.\n";
        } else if (!line.empty()) {
            std::cout << "[input] Unknown command. Use 'p' or 'q'.\n";
        }
    }
    st.running.store(false);
}

// Main Function
int main() {
    // adjust timings if you want
    Config cfg;          
    SharedState state;

    std::thread tController(controllerThread, std::ref(state), std::cref(cfg));
    std::thread tInput(inputThread, std::ref(state));

    tInput.join();
    state.running.store(false);
    tController.join();

    std::cout << "Shutting down cleanly.\n";
    return 0;
}
