## Description

Yada Yada this is a semester project we'll make this more official later i promise.

## Requirements
To install the required libraries, run the following command:
```bash
    pip install -r requirements.txt
```
- Note: The versions of the libraries listed in the requirements.txt file are compatible with Python3.11. However, compatibility with other Python versions has not been verified and may cause issues.

- We utilize a library called RTDE, which can be found under src/Robot. There is an additional README.md file in that directory with instructions on how to download this library. The `requirements.txt` file should be able to automatically download the library but if it causes any problems then consider running the commands that are found in `src/Robot/README.md`.

- Additionally, we have a UR script that needs to be uploaded to the robot's iPads to facilitate communication with the robot. we called these scripts  something like `rtde_simple_servoj` or `rtde_synched_servoj`. (In our lab, the scripts are already installed on the ipads)

## Useful Repositories

1. We used the following repository for generating ArUco markers, which we placed on our plate. While there are online tools available for generating ArUco markers, we preferred using this library:
    ```bash
    https://github.com/dogod621/OpenCVMarkerPrinter
    ```


## Tools

we used a plate the size of A4 page where we placed the ArUcos on top, and a red foam ball.

## Running The Project

1. Activate your virtual environment (if you have one):

    ```bash
    source venv/bin/activate
    ```
2. Navigate to the ROOT directory of the project.

3. Run the desired script to perform the task you desire. For example, to balance the ball on a plate, run:
    ```python
    python3 synchronized_balancing.py
    # for other tasks, change the file you're running for instance:
    #  python3 camera.py
    #  python3 synchronized_path_follow_main.py
    ```

4. Communicate with the robots:

    - Turn on the iPads and load the corresponding script for communication. The script is likely named something similar to `rtde_synched_servoj`.
    Hit play on both the task and camera assistant robot.

**Order of Operations:**

- First, run the Python script.
- Then, hit run on the iPads to enable communication with the robots.
- After completing the task, stop the scripts on the iPads, and terminate your Python script. to ensure no sudden movements occur

**Safety Recommendation:**
- When running the robots, keep your hand near the emergency stop buttons in case an incorrect configuration causes the robot to move in a dangerous manner.
