import os
import re

file_path = "road_conditions.py"
with open(file_path, "r") as f:
    content = f.read()

# Fix rough mode getMaxSpeed and setMaxSpeed
content = content.replace("orig_speed = self._traci.edge.getMaxSpeed(edge)", "orig_speed = self._traci.lane.getMaxSpeed(edge + '_0')")
content = content.replace("self._traci.edge.setMaxSpeed(edge, self.v_low)", "for i in range(self._traci.edge.getLaneNumber(edge)):\\n                        self._traci.lane.setMaxSpeed(edge + '_' + str(i), self.v_low)")

# Fix oscillate_rough
content = content.replace("self._traci.edge.setMaxSpeed(edge, safe_target)", "for i in range(self._traci.edge.getLaneNumber(edge)):\\n                            self._traci.lane.setMaxSpeed(edge + '_' + str(i), safe_target)")

# Fix deactivate
content = content.replace("self._traci.edge.setMaxSpeed(edge, orig_v)", "for i in range(self._traci.edge.getLaneNumber(edge)):\\n                        self._traci.lane.setMaxSpeed(edge + '_' + str(i), orig_v)")

with open(file_path, "w") as f:
    f.write(content)
print("Updated road_conditions.py TraCI lane APIs")
