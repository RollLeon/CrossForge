#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <tinyfsm.hpp>

struct ConversationEvent : tinyfsm::Event
{
	
};

struct Call : ConversationEvent { };


class StateMachine 
	: public tinyfsm::Fsm<Robot>
	{
	public:
		/* default reaction for unhandled events */
		void react(tinyfsm::Event const&) { };

		void react(Call        const&);

		void entry(void) { };  /* entry actions in some states */
		void         exit(void) { };  /* no exit actions */
};

	class Conversation
		: public Robot
	{
		void entry() override;
	};

	class Moving
		: public Robot
	{
		void react(FloorSensor const&) override;
	};


	void Idle::entry() {
		send_event(MotorStop());
	}

	void Idle::react(Call const& e) {
		dest_floor = e.floor;

		if (dest_floor == current_floor)
			return;

		/* lambda function used for transition action */
		auto action = [] {
			if (dest_floor > current_floor)
				send_event(MotorUp());
			else if (dest_floor < current_floor)
				send_event(MotorDown());
		};

		transit<Moving>(action);
	};
#endif //STATEMACHINE_H