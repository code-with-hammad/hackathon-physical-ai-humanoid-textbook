from omni.isaac.kit import SimulationApp

# This is a placeholder for actual Isaac Sim application startup.
# In a full Isaac Sim environment, this handles context creation.
simulation_app = SimulationApp({"headless": False})

import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

# Get the absolute path of the USD scene
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
scene_path = os.path.join(current_dir, "../environments/basic_scene.usd")

# Load the stage
stage = omni.usd.get_context().get_stage()
# Clear existing stage content before loading
omni.usd.get_context().new_stage()
omni.usd.get_context().open_stage(scene_path)

print(f"Loaded stage: {scene_path}")

# Play the simulation
timeline = omni.timeline.get_timeline_interface()
timeline.play()

# Keep the simulation running. In a real script, you'd add
# more complex logic, event handling, or a proper exit condition.
# For demonstration, we'll run for a few seconds.
import time
# time.sleep(10) # Removed for non-blocking execution in CLI.
# In a real setup, a proper simulation loop or event handling would be implemented.

# Do not stop the simulation_app in a CLI context as it will close the app
# simulation_app.update() until simulation is done or stopped by user.
# For a basic example, we will let the user close Isaac Sim manually.

print("Isaac Sim scene loaded and playing. Please close Isaac Sim manually when done.")

# The simulation_app.update() loop would typically go here
# while simulation_app.is_running():
#     simulation_app.update()

# Do not call simulation_app.shutdown() here if you want the app to remain open.
# simulation_app.shutdown()
