# IMU 相关文件具体位置（可直接复制路径）

下面是你问的两个“值”的准确位置，不需要点击链接。

一、`model.config` 里声明入口 `model.sdf` 的位置  
文件路径：  
`d:\eval\agentsd_ws\src\smart_agent_gazebo\models\smart_agent\model.config`  
行号：第 5 行  
内容：  
`<sdf version="1.9">model.sdf</sdf>`

二、`model.sdf` 里 IMU 传感器配置的位置  
文件路径：  
`d:\eval\agentsd_ws\src\smart_agent_gazebo\models\smart_agent\model.sdf`

关键行：  
第 44 行：  
`<sensor name="imu_sensor" type="imu">`

第 45 行：  
`<topic>/agent/imu</topic>`

第 46 行：  
`<update_rate>100</update_rate>`

三、world 里 IMU 系统插件的位置（补充）  
文件路径：  
`d:\eval\agentsd_ws\src\smart_agent_gazebo\worlds\small_house.world`  
行号：第 11 行  
内容：  
`<plugin filename="libignition-gazebo-imu-system.so" name="ignition::gazebo::systems::Imu"/>`

四、你可以在终端直接检查（Windows CMD）

查看 model.config：  
`type d:\eval\agentsd_ws\src\smart_agent_gazebo\models\smart_agent\model.config`

查看 model.sdf：  
`type d:\eval\agentsd_ws\src\smart_agent_gazebo\models\smart_agent\model.sdf`

搜索 IMU 行：  
`rg -n imu_sensor d:\eval\agentsd_ws\src\smart_agent_gazebo\models\smart_agent\model.sdf`  
`rg -n "/agent/imu" d:\eval\agentsd_ws\src\smart_agent_gazebo\models\smart_agent\model.sdf`  
`rg -n "<update_rate>100</update_rate>" d:\eval\agentsd_ws\src\smart_agent_gazebo\models\smart_agent\model.sdf`

