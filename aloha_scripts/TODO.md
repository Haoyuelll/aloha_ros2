# TODO

1. add safe sleep for robot arms when receiving the keyboard interupt
    
  **Current problem:**
    - In the launcher script:
      - The ROS core process dies immeriatly when the ctrl+c signal is given 
      - Then the robot cannot receive the commands no matter if the ROS node is still alive or not
      - Besides, the ros core process takes up the main thread and cannot be interupted with other threads 
    - In the teleop script:
      - The interbotix node partly dies the moment when the  ctrl+c signal is given
      - Then neither creating a new interbotix node or reusing the old one will not work
  
  **Possible solution:** use `signal` lib to hang the keyboard interupt and do cleanup (sample below)

  ```python
  import signal
  import time
  
  
  def setup_interrupt_handler():
      state = {"interrupted": False}
      def handle_interrupt(signum, frame):
          print("Ctrl+C detected! Running callback...")
          state["interrupted"] = True
          # Perform cleanup or other actions
          cleanup()
          print("Exiting gracefully.")
          exit(0)
      def cleanup():
          print("Performing cleanup tasks...")
    
      # Register the signal handler for SIGINT
      signal.signal(signal.SIGINT, handle_interrupt)
      return state
  
  def long_running_task():
      state = setup_interrupt_handler()
      print("Press Ctrl+C to trigger the callback...")
      while True:
          # Simulate long-running work
          time.sleep(1)
          print("Working...")
          if state["interrupted"]:
              print("Task interrupted. Exiting.")
              break
  
  
  if __name__ == "__main__":
      long_running_task()
  ```

2. Code cleanup
  - teleop duplicates:
    - `teleop.py`, `one_side_teleop.py`, `both_side_teleop.py`, `record_jas.py`
  - calibration duplicates:
    - `camera_calibration.py` and scripts under `calibration` folder
  - sleep duplicates:
    - multiple functions across teleop and calibration scripts, and a stand-alone sleep script
 