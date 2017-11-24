## SUTURO 1718 Motionpackage

### Installation

Das Repo in den src Ordner eines catkin workspaces clonen.
Das msgs_suturo_1718 package muss sich auch im src Ordner des workspaces befinden.
Dann den catkin workspace mit catkin build bauen.

### Actions

Folgende Actions gibt es:

#### MovingCommandAction

Über die MovingCommandAction wird unser Actionserver angesprochen.
Bestandteile der Action:

- geometry_msgs/PointStamped
- uint8 command

Folgende Konstanten sind für command vorgesehen

<table>
  <tr>
    <th>Constant</th>
    <th>int-value</th>
    <th>Bedeutung</th>
  </tr>
</table>

- UNKNOWN=0
- MOVE_STANDARD_POSE=1 (Roboter Arme in die Initale Pose bewegen, kein PointStamped benötigt)
- MOVE_RIGHT_ARM=2 (Rechten Arm zu gegebenem PointStamped bewegen)
- MOVE_LEFT_ARM=3 (Linken Arm zu gegebenem PointStamped bewegen)

Folgends wird zurückgegeben:

- bool successfull (true/false, je nachdem ob die Bewegung durchgeführt wurde oder nicht)



