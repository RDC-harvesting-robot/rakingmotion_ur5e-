# UR5eによるかき分け動作 
# 必要なパッケージ
- UR5e (https://github.com/open-rdc/harvesting_robot/issues/199)
- leptrino_force_sensor (https://github.com/open-rdc/leptrino_force_sensor/tree/humble_devel)

# 実行方法

```
ros2 launch rakingmotion_ur5e rakingmotion_with_force_sensor.launch.py direction:=L
```

