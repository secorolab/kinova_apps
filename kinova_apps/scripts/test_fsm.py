import signal
import sys
from dataclasses import dataclass
from coord_dsl.event_loop import (
    produce_event,
    consume_event,
    reconfig_event_buffers,
)
from coord_dsl.fsm import FSMData, fsm_step

from fsm_kinova_sorting import (
    EventID,
    StateID,
    create_fsm
)

def signal_handler(sig, frame):
    print("You pressed Ctrl+C! Exiting gracefully...")
    sys.exit(0)


@dataclass
class UserData:
    counter:    int    = 0
    first_dets: bool   = True
    num_sorted: int    = 0
    max_sort:   int    = 3



def configure_step(fsm: FSMData, ud: UserData):
    if consume_event(fsm.event_data, EventID.E_CONFIGURE_ENTER):
        print(f"Entered state '{StateID(fsm.current_state_index).name}'")
 
    while ud.counter < 50:
        ud.counter += 1
        return False

    print(f'Configured after {ud.counter} steps')
    ud.counter = 0
    return True


def idle_step(fsm: FSMData, ud: UserData):
    if consume_event(fsm.event_data, EventID.E_IDLE_ENTER):
        print(f"Entered state '{StateID(fsm.current_state_index).name}'")

    if consume_event(fsm.event_data, EventID.E_SORTING_EXIT_IDLE_ENTER):
        print("Returned to idle from sorting")
        produce_event(fsm.event_data, EventID.E_IDLE_EXIT)

    ud.counter += 1

    if ud.counter == 20:
        print(f'Leaving idle after {ud.counter} steps')
        ud.counter = 0
        return True
    
    if consume_event(fsm.event_data, EventID.E_CONFIGURE_IDLE):
        print("Configure to idle transition")
        ud.counter = 50
        return True

    if ud.counter >= 50:
        print(f'Idle timeout after {ud.counter} steps, going to home arm')
        ud.counter = 0
        produce_event(fsm.event_data, EventID.E_IDLE_HOME_ARM)

    return False


def home_arm_step(fsm: FSMData, ud: UserData):
    if consume_event(fsm.event_data, EventID.E_HOME_ARM_ENTER):
        print(f"Entered state '{StateID(fsm.current_state_index).name}'")

    while ud.counter < 30:
        ud.counter += 1
        return False

    print(f'Arm homed after {ud.counter} steps')
    ud.counter = 0

    return True


def sorting_step(fsm: FSMData, ud: UserData):
    if consume_event(fsm.event_data, EventID.E_SORTING_ENTER):
        print(f"Entered state '{StateID(fsm.current_state_index).name}'")
        ud.num_sorted = 0
        print('Detecting objects for the first time')
        produce_event(fsm.event_data, EventID.E_SORTING_DETECT_OBJECTS)
        return False

    if ud.num_sorted >= ud.max_sort:
        print(f'Sorted {ud.num_sorted} objects, exiting sorting')
        return True

    if consume_event(fsm.event_data, EventID.E_DETECT_OBJECTS_SORTING):
        print(f"Returned to sorting from detect objects, sorted {ud.num_sorted} objects")
        produce_event(fsm.event_data, EventID.E_PICK_OBJECT)

    if consume_event(fsm.event_data, EventID.E_PLACE_OBJECT_SORTING):
        ud.num_sorted += 1
        print(f"Returned to sorting from place object, sorted {ud.num_sorted} objects")
        if ud.num_sorted < ud.max_sort:
            produce_event(fsm.event_data, EventID.E_SORTING_DETECT_OBJECTS)

    return False


def detect_objects_step(fsm: FSMData, ud: UserData):
    if consume_event(fsm.event_data, EventID.E_DETECT_OBJECTS_ENTER):
        print(f"Entered state '{StateID(fsm.current_state_index).name}'")

    while ud.counter < 40:
        ud.counter += 1
        return False

    print(f'Detected objects after {ud.counter} steps')
    ud.counter = 0

    return True


def pick_object_step(fsm: FSMData, ud: UserData):
    if consume_event(fsm.event_data, EventID.E_PICK_OBJECT_ENTER):
        print(f"Entered state '{StateID(fsm.current_state_index).name}'")

    while ud.counter < 60:
        ud.counter += 1
        return False

    print(f'Picked object after {ud.counter} steps')
    ud.counter = 0

    return True


def place_object_step(fsm: FSMData, ud: UserData):
    if consume_event(fsm.event_data, EventID.E_PLACE_OBJECT_ENTER):
        print(f"Entered state '{StateID(fsm.current_state_index).name}'")

    while ud.counter < 60:
        ud.counter += 1
        return False

    print(f'Placed object after {ud.counter} steps')
    ud.counter = 0

    return True


def generic_on_end(fsm: FSMData, ud: UserData, end_events: list[EventID]):
    print(f"State '{StateID(fsm.current_state_index).name}' finished")
    for evt in end_events:
        produce_event(fsm.event_data, evt)

def fsm_behavior(fsm: FSMData, ud: UserData, bhv_data: dict):
    cs = fsm.current_state_index
    if cs not in bhv_data:
        return

    bhv_data_cs = bhv_data[cs]

    assert "step" in bhv_data_cs, f"no step defined for state: {cs}"
    if not bhv_data_cs["step"](fsm, ud):
        return

    if "on_end" in bhv_data_cs:
        bhv_data_cs["on_end"](fsm, ud)


def main():
    signal.signal(signal.SIGINT, signal_handler)

    # Create FSM
    fsm = create_fsm()

    fsm_bhv = {
        StateID.S_CONFIGURE: {
            "step": configure_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_CONFIGURE_EXIT]
            ),
        },
        StateID.S_IDLE: {
           "step": idle_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_STEP]
            ),
        },
        StateID.S_HOME_ARM: {
            "step": home_arm_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_HOME_ARM_EXIT]
            ),
        },
        StateID.S_SORTING: {
            "step": sorting_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_SORTING_EXIT]
            ),
        },
        StateID.S_DETECT_OBJECTS: {
            "step": detect_objects_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_DETECT_OBJECTS_EXIT]
            ),
        },
        StateID.S_PICK_OBJECT: {
            "step": pick_object_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_PICK_OBJECT_EXIT]
            ),
        },
        StateID.S_PLACE_OBJECT: {
            "step": place_object_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_PLACE_OBJECT_EXIT]
            ),
        },
    }


    ud = UserData()

    while True:
        if fsm.current_state_index == StateID.S_EXIT:
            print("State machine completed successfully")
            break

        produce_event(fsm.event_data, EventID.E_STEP)
        fsm_behavior(fsm, ud, fsm_bhv)
        reconfig_event_buffers(fsm.event_data)
        
        fsm_step(fsm)
        reconfig_event_buffers(fsm.event_data)
        
        # time.sleep(0.01)

    
if __name__ == "__main__":
    main()
