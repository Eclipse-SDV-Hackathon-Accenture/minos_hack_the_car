import open3d as o3d
import numpy as np
import time
from threading import Thread, Event

counter = 1

event = Event()


# Create a point cloud
point_cloud = o3d.io.read_point_cloud("../data/frames2/test0.pcd")


# Function to update point cloud (replace this with your logic)
def update_point_cloud():
    global counter, point_cloud
    # Example: Randomly move points
    point_cloud1 = o3d.io.read_point_cloud("../data/frames2/test" + str(10 * counter) + ".pcd")

    point_cloud.points = point_cloud1.points
    counter += 1


def update_ui():
    global counter, point_cloud
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(point_cloud)

    while True:

        if event.is_set():
            point_cloud1 = o3d.io.read_point_cloud("../data/frames2/test" + str(10 * counter) + ".pcd")

            point_cloud.points = point_cloud1.points
            counter += 1
            event.clear()

        vis.update_geometry(point_cloud)
        vis.poll_events()
        vis.update_renderer()

        # Optionally, add a small delay to control the frame rate
        if counter > 4:
            counter = 0


def main():
    thread = Thread(target=update_ui, name="UI Updater")
    thread.start()
    print(1000)

    while True:
        time.sleep(1)

        event.set()
        print(event.is_set())


if __name__ == "__main__":
    main()
