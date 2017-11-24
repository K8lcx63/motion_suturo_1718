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

<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Bedeutung</th>
  </tr>
  <tr>
    <td>goal_point</td>
    <td>geometry_msgs/PointStamped</td>
    <td>Der Endpunkt der Bewegung als PointStamped</td>
  </tr>
  <tr>
    <td>command</td>
    <td>uint8</td>
    <td>Konstante, die die Art des Befehles definiert (siehe unten)</td>
  </tr>
</table>

Folgende Konstanten sind für command vorgesehen

<table>
  <tr>
    <th>Constant</th>
    <th>Int-value</th>
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
    <th>Type</th>
    <th>Name</th>
    <th>Bedeutung</th>
  </tr>
  <tr>
    <td>bool</td>
    <td>successfull</td>
    <td>true/false, je nachdem ob die Bewegung erfolgreich durchgeführt werden konnte oder nicht</td>
  </tr>
</table>