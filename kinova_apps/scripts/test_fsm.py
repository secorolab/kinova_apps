import signal
import sys
from dataclasses import dataclass
import time
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
    exit_flag:  bool   = False


def wait_step(ud: UserData):
    if ud.counter < 1:
        # time.sleep(0.01)
        ud.counter += 1
        return True
    ud.counter = 0
    return True

def configure_step(fsm: FSMData, ud: UserData):
    if consume_event(fsm.event_data, EventID.E_CONFIGURE_ENTER):
        print(f"Entered state '{StateID(fsm.current_state_index).name}'")
        
    time.sleep(4)

    return True

def idle_step(fsm: FSMData, ud: UserData):
    if consume_event(fsm.event_data, EventID.E_IDLE_ENTER):
        print(f"Entered state '{StateID(fsm.current_state_index).name}'")

    return True


def generic_on_end(fsm: FSMData, ud: UserData, end_events: list[EventID]):
    print(f"State '{StateID(fsm.current_state_index).name}' finished")
    for evt in end_events:
        produce_event(fsm.event_data, evt)

def generic_on_start(fsm: FSMData, ud: UserData, start_event: EventID):
    if consume_event(fsm.event_data, start_event):
        print(f"Entered state '{StateID(fsm.current_state_index).name}'")

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
                fsm, ud, [EventID.E_IDLE_EXIT]
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
