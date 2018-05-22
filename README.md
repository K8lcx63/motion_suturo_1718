
## Wiki zum Paket _motion_suturo_1718_

### Installation

MoveIt installieren:
> sudo apt-get install ros-indigo-moveit

> source /opt/ros/indigo/setup.bash

Das Repository [motion_suturo_1718](https://github.com/menanuni/motion_suturo_1718.git), [vision_suturo_1718](https://github.com/menanuni/vision_suturo_1718.git), [knowledge_suturo_1718](https://github.com/menanuni/knowledge_suturo_1718.git), [msgs_suturo_1718](https://github.com/menanuni/msgs_suturo_1718.git) und [common_suturo1718](https://github.com/menanuni/common_suturo1718.git) in den _src_-Ordner eines _catkin-workspaces_ clonen.

Dann den _catkin-workspace_ mit 
> catkin build 

bauen.

### Ausführung

Das Paket kann dann über folgende Befehle gestartet werden:

Für das Ausführen in der Simulation:
> roslaunch motion motion_main_start.launch

Für das Ausführen am echten Roboter:
> roslaunch kitchen_model_export knowledge_export_service.launch
> 
>  roslaunch motion motion_main_start_real_pr2.launch

### Angebotene Actions

Folgende Action gibt es im Paket _motion_suturo_1718_:

##### _~/moving_ vom Typ _motion_msgs/MovingCommand_

###### Goal

Über die MovingCommandAction wird unserem Actionserver ein zu erreichendes Ziel vorgegeben.
Bestandteile der Nachricht:

<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Bedeutung</th>
  </tr>
  <tr>
    <td>goal_pose</td>
    <td>geometry_msgs/PoseStamped</td>
    <td>Enthält eine Position und Orientierung, die abhängig vom Kommando, das in 'command' angegeben wird, interpretiert werden. Siehe weiter unten für genauere Erklärung.</td>
  </tr>
  <tr>
    <td>goal_poses</td>
    <td>geometry_msgs/PoseArray</td>
    <td>Enthält eine Liste von Greifposen, die benutzt wird, wenn in 'command' der Befehl zum Greifen eines Objektes gegeben wird. Siehe weiter unten für genauere Erklärung.</td>
  </tr>
  <tr>
    <td>command</td>
    <td>uint8</td>
    <td>Variable, die die Art des Befehles definiert. Siehe weiter unten für definierte Konstanten für diese Variable.</td>
  </tr>
  <tr>
    <td>force</td>
    <td>float64</td>
    <td>Variable, die die Kraft angibt, mit der gegriffen werden soll. Nur benötigt, falls ein Kommando zum Greifen abgesetzt wird. Wird dieser Wert bei einem Kommando zum Greifen nicht befüllt, wird ein default-Wert verwendet.</td>
  </tr>
  <tr>
    <td>grasped_object_label</td>
    <td>string</td>
    <td>Variable, die den Namen des zu greifenden, abzustellenden oder zu verschiebenden Objektes enthält.</td>
  </tr>
</table>

Folgende Konstanten kann die Variable _command_ annehmen:

<table>
  <tr>
    <th>Name</th>
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
    <td>Arme in die Initiale Pose bewegen, kein PoseStamped benötigt.</td>
  </tr>
  <tr>
    <td>MOVE_RIGHT_ARM</td>
    <td>2</td>
    <td>Rechten Arm zu gegebenem Punkt mit gegebener Orientierung bewegen.</td>
  </tr>
  <tr>
    <td>MOVE_LEFT_ARM</td>
    <td>3</td>
    <td>Linken Arm zu gegebenem Punkt mit gegebener Orientierung bewegen.</td>
  </tr>
  <tr>
    <td>POKE_RIGHT_ARM</td>
    <td>4</td>
    <td>Mit rechtem Arm Objekt an gegebener Position mit gegebener Orientierung des Armes umstoßen.</td>
  </tr>
  <tr>
    <td>POKE_LEFT_ARM</td>
    <td>5</td>
    <td>Mit linkem Arm Objekt an gegebener Position mit gegebener Orientierung des Armes umstoßen.</td>
  </tr>
  <tr>
    <td>GRASP_RIGHT_ARM</td>
    <td>6</td>
    <td>Mit rechtem Arm das Objekt mit einer der übergebenen Greifposen greifen. Die beste Greifpose wird automatisch selektiert.</td>
  </tr>
  <tr>
    <td>GRASP_LEFT_ARM</td>
    <td>7</td>
    <td>Mit linkem Arm das Objekt mit einer der übergebenen Greifposen greifen. Die beste Greifpose wird automatisch selektiert.</td>
  </tr>
  <tr>
    <td>PLACE_RIGHT_ARM</td>
    <td>8</td>
    <td>Mit rechtem Arm Objekt an gegebener Position mit gegebener Orientierung des Armes abstellen.</td>
  </tr>
  <tr>
    <td>PLACE_LEFT_ARM</td>
    <td>9</td>
    <td>Mit linkem Arm Objekt an gegebener Position mit gegebener Orientierung des Armes abstellen.</td>
  </tr>
  <tr>
    <td>MOVE_CARRY_POSE</td>
    <td>10</td>
    <td>Bewegen der Arme in eine Position, die zum Fahren und gleichzeitig zum Tragen von Objekten geeignet ist.</td>
  </tr>
  <tr>
    <td>MOVE_CARRY_POSE_RIGHT</td>
    <td>10</td>
    <td>Bewegen des rechten Armes in eine Position, die zum Fahren und gleichzeitig zum Tragen von Objekten geeignet ist.</td>
  </tr>
  <tr>
    <td>MOVE_CARRY_POSE_LEFT</td>
    <td>11</td>
    <td>Bewegen des linken Armes in eine Position, die zum Fahren und gleichzeitig zum Tragen von Objekten geeignet ist.</td>
  </tr>
</table>

###### Result

<table>
  <tr>
    <th>Type</th>
    <th>Name</th>
    <th>Bedeutung</th>
  </tr>
  <tr>
    <td>bool</td>
    <td>successful</td>
    <td>Gibt an, ob die Aktion erfolgreich ausgeführt werden konnte.</td>
  </tr>
  <tr>
    <td>uint8</td>
    <td>status</td>
    <td>Enthält das Ergebnis der Aktion die ausgeführt werden sollte, gibt dabei den genaueren Fehler an, falls die Aktion nicht ausgeführt werden konnte. Siehe weiter unten für definierte Konstanten für diese Variable.</td>
  </tr>
</table>

Die Rückgabe _status_ kann Werte folgender Konstanten annehmen:

<table>
  <tr>
    <th>Name</th>
    <th>Bedeutung</th>
  </tr>
  <tr>
    <td>SUCCESS</td>
    <td>Die Aktion wurde erfolgreich ausgeführt.</td>
  </tr>
  <tr>
    <td>OUT_OF_RANGE</td>
    <td>Die Aktion konnte nicht ausgeführt werden, weil das Ziel, bzw. eines der Teilziele auf dem Weg zum Ziel außer Reichweite war.</td>
  </tr>
  <tr>
    <td>COLLISION</td>
    <td>Die Aktion konnte nicht ausgeführt werden, weil die Zielposition, bzw. einer der Wegpunkte auf dem Weg zu der Zielposition in Kollision mit dem Roboter wäre.</td>
  </tr>
  <tr>
    <td>UNMANAGEBLE_ERROR</td>
    <td>Die Aktion konnte nicht ausgeführt werden, aufgrund eines internen Fehlers, der nicht von dem Aufrufenden durch Eingabe eines anderen Zieles behoben werden kann.</td>
  </tr>
</table>

##### Absetzen eines Kommandos an den _Actionserver_

Um unseren _Actionserver_ über die Konsole aufrufen zu können, können folgende zwei Befehle verwendet werden:

Für Kommando-Oberfläche mit GUI:
> rosrun actionlib axclient.py /moving

Für ein Kommando direkt über die Konsole:
> rostopic pub /moving/goal motion_msgs/MovingCommandActionGoal _GOAL_

Hier muss die Stelle, an der der Platzhalter _GOAL_ steht, noch durch ein gültiges _Action-Goal_ gemäß der oben angegebenen Beschreibung eingesetzt werden.

_
