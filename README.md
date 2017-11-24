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
  <tr>
    <td>UNKNOWN</td>
    <td>0</td>
    <td>-</td>
  </tr>
  <tr>
    <td>MOVE_STANDARD_POSE</td>
    <td>1</td>
    <td>Roboter Arme in die Initiale Pose bewegen, kein PointStamped benötigt</td>
  </tr>
  <tr>
    <td>MOVE_RIGHT_ARM</td>
    <td>2</td>
    <td>Rechten Arm zu gegebenem PointStamped bewegen</td>
  </tr>
  <tr>
    <td>MOVE_LEFT_ARM</td>
    <td>3</td>
    <td>Linken Arm zu gegebenem PointStamped bewegen</td>
  </tr>
</table>

Folgends wird zurückgegeben:

<table>
  <tr>
    <th>type</th>
    <th>name</th>
    <th>definition</th>
  </tr>
  <tr>
    <td>bool</td>
    <td>successfull</td>
    <td>true/false, je nachdem ob die Bewegung erfolgreich durchgeführt werden konnte oder nicht</td>
  </tr>
</table>