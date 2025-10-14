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
    E_PICK_OBJECT = auto()
    E_PICK_OBJECT_ENTER = auto()
    E_PICK_OBJECT_EXIT = auto()
    E_PLACE_OBJECT_ENTER = auto()
    E_PLACE_OBJECT_EXIT = auto()
    E_PLACE_OBJECT_SORTING = auto()
    E_SORTING_EXIT_IDLE_ENTER = auto()
    E_STEP = auto()


# State IDs
class StateID(IntEnum):
    S_START = 0
    S_CONFIGURE = auto()
    S_IDLE = auto()
    S_HOME_ARM = auto()
    S_DETECT_OBJECTS = auto()
    S_SORTING = auto()
    S_PICK_OBJECT = auto()
    S_PLACE_OBJECT = auto()
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
    T_SORTING_PICK_OBJECT = auto()
    T_PICK_OBJECT_PLACE_OBJECT = auto()
    T_PLACE_OBJECT_SORTING = auto()
    T_SORTING_IDLE = auto()
    T_IDLE_EXIT = auto()


# Event reaction IDs
class ReactionID(IntEnum):
    R_E_CONFIGIRE_EXIT = 0
    R_E_IDLE_HOME_ARM = auto()
    R_E_HOME_ARM_EXIT = auto()
    R_E_SORTING_DETECT_OBJECTS = auto()
    R_E_DETECT_OBJECTS_EXIT = auto()
    R_E_SORTING_PICK_OBJECT = auto()
    R_E_PICK_OBJECT_EXIT = auto()
    R_E_PLACE_OBJECT_EXIT = auto()
    R_E_SORTING_EXIT = auto()
    R_E_IDLE_EXIT = auto()
    R_E_STEP1 = auto()
    R_E_STEP2 = auto()


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
        TransitionID.T_SORTING_PICK_OBJECT: Transition(StateID.S_SORTING, StateID.S_PICK_OBJECT),
        TransitionID.T_PICK_OBJECT_PLACE_OBJECT: Transition(StateID.S_PICK_OBJECT, StateID.S_PLACE_OBJECT),
        TransitionID.T_PLACE_OBJECT_SORTING: Transition(StateID.S_PLACE_OBJECT, StateID.S_SORTING),
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
        ReactionID.R_E_SORTING_PICK_OBJECT: EventReaction(
            condition_event_index=EventID.E_PICK_OBJECT,
            transition_index=TransitionID.T_SORTING_PICK_OBJECT,
            fired_event_indices=[
                EventID.E_PICK_OBJECT_ENTER,
            ],
        ),
        ReactionID.R_E_PICK_OBJECT_EXIT: EventReaction(
            condition_event_index=EventID.E_PICK_OBJECT_EXIT,
            transition_index=TransitionID.T_PICK_OBJECT_PLACE_OBJECT,
            fired_event_indices=[
                EventID.E_PLACE_OBJECT_ENTER,
            ],
        ),
        ReactionID.R_E_PLACE_OBJECT_EXIT: EventReaction(
            condition_event_index=EventID.E_PLACE_OBJECT_EXIT,
            transition_index=TransitionID.T_PLACE_OBJECT_SORTING,
            fired_event_indices=[
                EventID.E_PLACE_OBJECT_SORTING,
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