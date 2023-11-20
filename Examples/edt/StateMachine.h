#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <iostream>
#include <tinyfsm.hpp>

// Forward declarations for state classes
class Talking;
class Behavior;

// Event to trigger state transitions
struct ToggleEvent : tinyfsm::Event {};

// Base class for states
class Robot : public tinyfsm::State {
public:
    virtual void entry() { }
};

// State: Talking
class Talking : public Robot {
public:
    void entry() override {
        std::cout << "Entered state: Talking\n" << std::endl;
    }
};

// State: Behavior
class Behavior : public Robot {
public:
    void entry() override {
        std::cout << "Entered state: Behavior\n" << std::endl;
    }
};

// Define the finite state machine
class MyFSM : public tinyfsm::Fsm<MyFSM> {
public:
    // Initial state
    void react(Tiny::Initial const&) {
        std::cout << "Initial state: Behavior\n"<<std::endl;
        transit<Behavior>();
    }

    void react(ToggleEvent const&) {
        std::cout << "Toggle event received. " << std::endl;
        if (is<Talking>()) {
            std::cout << "Transitioning from Talking to Behavior\n" << std::endl;
            transit<Behavior>();
        }
        else if (is<Behavior>()) {
            std::cout << "Transitioning from Behavior to Talking\n" << std::endl;
            transit<Talking>();
        }
    }
};

#endif //STATEMACHINE_H