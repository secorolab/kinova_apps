"""
This is an auto-generated file. Do not edit it directly.

FSM: kinova_sorting
FSM Description: A finite state machine for sorting cubes and spheres using a Kinova robotic arm.

Examples:

>>> from coord_dsl.fsm import fsm_step
>>> from coord_dsl.event_loop import reconfig_event_buffers
>>> from fsm_example import create_fsm
>>> fsm = create_fsm()
>>> while True:
...     if fsm.current_state_index == StateID.S_EXIT:
...         print("State machine completed successfully")
...         break
...     fsm_behavior(fsm, ud) # user-defined behaviour with user data
...     fsm_step(fsm)
...     reconfig_event_buffers(fsm.event_data)
"""
from enum import IntEnum, auto
from coord_dsl.event_loop import EventData
from coord_dsl.fsm import FSMData, Transition, EventReaction


# Event IDs
class EventID(IntEnum):
    E_CONFIGURE_ENTER = 0
    E_CONFIGURE_EXIT = auto()
    E_CONFIGURE_IDLE = auto()
    E_IDLE_ENTER = auto()
    E_IDLE_EXIT = auto()
    E_IDLE_HOME_ARM = auto()
    E_HOME_ARM_ENTER = auto()
    E_HOME_ARM_EXIT = auto()
    E_SORTING_DETECT_OBJECTS = auto()
    E_DETECT_OBJECTS_EXIT = auto()
    E_DETECT_OBJECTS_ENTER = auto()
    E_DETECT_OBJECTS_SORTING = auto()
    E_SORTING_ENTER = auto()
    E_SORTING_EXIT = auto()
    E_MOVE_ARM = auto()
    E_MOVE_ARM_ENTER = auto()
    E_MOVE_ARM_EXIT = auto()
    E_GRIPPER_CONTROL = auto()
    E_GRIPPER_CONTROL_ENTER = auto()
    E_GRIPPER_CONTROL_EXIT = auto()
    E_GC_SORTING_ENTER = auto()
    E_SORTING_EXIT_IDLE_ENTER = auto()
    E_WAIT = auto()
    E_CONTINUE = auto()
    E_WAIT_ENTER = auto()
    E_WAIT_EXIT = auto()
    E_STEP = auto()


# State IDs
class StateID(IntEnum):
    S_START = 0
    S_CONFIGURE = auto()
    S_IDLE = auto()
    S_HOME_ARM = auto()
    S_DETECT_OBJECTS = auto()
    S_SORTING = auto()
    S_MOVE_ARM = auto()
    S_GRIPPER_CONTROL = auto()
    S_WAIT = auto()
    S_EXIT = auto()


# Transition IDs
class TransitionID(IntEnum):
    T_START_CONFIGURE = 0
    T_CONFIGURE_IDLE = auto()
    T_IDLE_IDLE = auto()
    T_IDLE_HOME_ARM = auto()
    T_HOME_ARM_SORTING = auto()
    T_SORTING_DETECT_OBJECTS = auto()
    T_DETECT_OBJECTS_SORTING = auto()
    T_SORTING_MOVE_ARM = auto()
    T_MOVE_ARM_GRIPPER_CONTROL = auto()
    T_GC_SORTING = auto()
    T_SORTING_WAIT = auto()
    T_WAIT_WAIT = auto()
    T_WAIT_SORTING = auto()
    T_SORTING_IDLE = auto()
    T_IDLE_EXIT = auto()


# Event reaction IDs
class ReactionID(IntEnum):
    R_E_CONFIGIRE_EXIT = 0
    R_E_IDLE_HOME_ARM = auto()
    R_E_HOME_ARM_EXIT = auto()
    R_E_SORTING_DETECT_OBJECTS = auto()
    R_E_DETECT_OBJECTS_EXIT = auto()
    R_E_SORTING_MOVE_ARM = auto()
    R_E_MOVE_ARM_EXIT = auto()
    R_E_GRIPPER_CONTROL_EXIT = auto()
    R_E_SORTING_EXIT = auto()
    R_E_IDLE_EXIT = auto()
    R_E_WAIT = auto()
    R_E_WAIT_EXIT = auto()
    R_E_STEP1 = auto()
    R_E_STEP2 = auto()
    R_E_STEP3 = auto()


def create_fsm() -> FSMData:
    """Creates the FSM data structure."""
    # Transitions
    trans_dict = {
        TransitionID.T_START_CONFIGURE: Transition(StateID.S_START, StateID.S_CONFIGURE),
        TransitionID.T_CONFIGURE_IDLE: Transition(StateID.S_CONFIGURE, StateID.S_IDLE),
        TransitionID.T_IDLE_IDLE: Transition(StateID.S_IDLE, StateID.S_IDLE),
        TransitionID.T_IDLE_HOME_ARM: Transition(StateID.S_IDLE, StateID.S_HOME_ARM),
        TransitionID.T_HOME_ARM_SORTING: Transition(StateID.S_HOME_ARM, StateID.S_SORTING),
        TransitionID.T_SORTING_DETECT_OBJECTS: Transition(StateID.S_SORTING, StateID.S_DETECT_OBJECTS),
        TransitionID.T_DETECT_OBJECTS_SORTING: Transition(StateID.S_DETECT_OBJECTS, StateID.S_SORTING),
        TransitionID.T_SORTING_MOVE_ARM: Transition(StateID.S_SORTING, StateID.S_MOVE_ARM),
        TransitionID.T_MOVE_ARM_GRIPPER_CONTROL: Transition(StateID.S_MOVE_ARM, StateID.S_GRIPPER_CONTROL),
        TransitionID.T_GC_SORTING: Transition(StateID.S_GRIPPER_CONTROL, StateID.S_SORTING),
        TransitionID.T_SORTING_WAIT: Transition(StateID.S_SORTING, StateID.S_WAIT),
        TransitionID.T_WAIT_WAIT: Transition(StateID.S_WAIT, StateID.S_WAIT),
        TransitionID.T_WAIT_SORTING: Transition(StateID.S_WAIT, StateID.S_SORTING),
        TransitionID.T_SORTING_IDLE: Transition(StateID.S_SORTING, StateID.S_IDLE),
        TransitionID.T_IDLE_EXIT: Transition(StateID.S_IDLE, StateID.S_EXIT),
    }
    trans_list = [trans_dict[i] for i in TransitionID]

    # Event Reactions
    evt_reaction_dict = {
        ReactionID.R_E_CONFIGIRE_EXIT: EventReaction(
            condition_event_index=EventID.E_CONFIGURE_EXIT,
            transition_index=TransitionID.T_CONFIGURE_IDLE,
            fired_event_indices=[
                EventID.E_CONFIGURE_IDLE,
            ],
        ),
        ReactionID.R_E_IDLE_HOME_ARM: EventReaction(
            condition_event_index=EventID.E_IDLE_HOME_ARM,
            transition_index=TransitionID.T_IDLE_HOME_ARM,
            fired_event_indices=[
                EventID.E_HOME_ARM_ENTER,
            ],
        ),
        ReactionID.R_E_HOME_ARM_EXIT: EventReaction(
            condition_event_index=EventID.E_HOME_ARM_EXIT,
            transition_index=TransitionID.T_HOME_ARM_SORTING,
            fired_event_indices=[
                EventID.E_SORTING_ENTER,
            ],
        ),
        ReactionID.R_E_SORTING_DETECT_OBJECTS: EventReaction(
            condition_event_index=EventID.E_SORTING_DETECT_OBJECTS,
            transition_index=TransitionID.T_SORTING_DETECT_OBJECTS,
            fired_event_indices=[
                EventID.E_DETECT_OBJECTS_ENTER,
            ],
        ),
        ReactionID.R_E_DETECT_OBJECTS_EXIT: EventReaction(
            condition_event_index=EventID.E_DETECT_OBJECTS_EXIT,
            transition_index=TransitionID.T_DETECT_OBJECTS_SORTING,
            fired_event_indices=[
                EventID.E_DETECT_OBJECTS_SORTING,
            ],
        ),
        ReactionID.R_E_SORTING_MOVE_ARM: EventReaction(
            condition_event_index=EventID.E_MOVE_ARM,
            transition_index=TransitionID.T_SORTING_MOVE_ARM,
            fired_event_indices=[
                EventID.E_MOVE_ARM_ENTER,
            ],
        ),
        ReactionID.R_E_MOVE_ARM_EXIT: EventReaction(
            condition_event_index=EventID.E_MOVE_ARM_EXIT,
            transition_index=TransitionID.T_MOVE_ARM_GRIPPER_CONTROL,
            fired_event_indices=[
                EventID.E_GRIPPER_CONTROL_ENTER,
            ],
        ),
        ReactionID.R_E_GRIPPER_CONTROL_EXIT: EventReaction(
            condition_event_index=EventID.E_GRIPPER_CONTROL_EXIT,
            transition_index=TransitionID.T_GC_SORTING,
            fired_event_indices=[
                EventID.E_GC_SORTING_ENTER,
            ],
        ),
        ReactionID.R_E_SORTING_EXIT: EventReaction(
            condition_event_index=EventID.E_SORTING_EXIT,
            transition_index=TransitionID.T_SORTING_IDLE,
            fired_event_indices=[
                EventID.E_SORTING_EXIT_IDLE_ENTER,
            ],
        ),
        ReactionID.R_E_IDLE_EXIT: EventReaction(
            condition_event_index=EventID.E_IDLE_EXIT,
            transition_index=TransitionID.T_IDLE_EXIT,
            fired_event_indices=[],
        ),
        ReactionID.R_E_WAIT: EventReaction(
            condition_event_index=EventID.E_WAIT,
            transition_index=TransitionID.T_SORTING_WAIT,
            fired_event_indices=[
                EventID.E_WAIT_ENTER,
            ],
        ),
        ReactionID.R_E_WAIT_EXIT: EventReaction(
            condition_event_index=EventID.E_WAIT_EXIT,
            transition_index=TransitionID.T_WAIT_SORTING,
            fired_event_indices=[
                EventID.E_CONTINUE,
            ],
        ),
        ReactionID.R_E_STEP1: EventReaction(
            condition_event_index=EventID.E_STEP,
            transition_index=TransitionID.T_START_CONFIGURE,
            fired_event_indices=[
                EventID.E_CONFIGURE_ENTER,
            ],
        ),
        ReactionID.R_E_STEP2: EventReaction(
            condition_event_index=EventID.E_STEP,
            transition_index=TransitionID.T_IDLE_IDLE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_STEP3: EventReaction(
            condition_event_index=EventID.E_STEP,
            transition_index=TransitionID.T_WAIT_WAIT,
            fired_event_indices=[],
        ),
    }
    evt_reaction_list = [evt_reaction_dict[i] for i in ReactionID]

    # Events
    events = EventData(len(EventID))

    # Return FSM Data
    return FSMData(
        event_data=events,
        num_states=len(StateID),
        start_state_index=StateID.S_START,
        end_state_index=StateID.S_EXIT,
        transitions=trans_list,
        event_reactions=evt_reaction_list,
        current_state_index=StateID.S_START,
    )