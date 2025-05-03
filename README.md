# ProgettoTIAGoRobotica

## Overview del Progetto
### Obiettivo:
Con questo Progetto si è sviluppato un framework di simulazione che consente al robot TIAGo di eseguire attività di pick and place, sfruttando algoritmi avanzati basati sull'intelligenza artificiale per la modellazione, la pianificazione del movimento, il controllo e la percezione del robot.

### Tools & conoscenze richieste per lo sviluppo del Progetto:
ROS 2, simulatore Rviz e Gazebo, Basi di Robotica, Intelligenza Artificiale e ArUcomarker.

###Sfide principali:
*Task 1:* 
	Modellazione di robot in Gazebo
	Localizzazione di oggetti
	Trasformazioni tra frame
*Task 2:* 
	Pianificazione del movimento
	Inversione cinematica
	Controllo del movimento
*Task 3:*
	Uso di AI in uno dei blocchi funzionali sviluppati nel task 2

## Specifiche Task 1:
Programmare il robot TIAGo per I) eseguire una scansione dell'ambiente muovendo gradualmente la testa del robot; II) rilevare i marker ArUco tramite la camera RGB-D e III) trasformare le coordinate dei marker dal sistema di riferimento della camera al sistema di riferimento della base del robot.

### Step seguiti:
	- Avviare la simulazione del robot in Gazebo
	- Movimentare la testa del TIAGo al fine di scansionare l’ambiente
	- Rilevare la presenza di ArUcomarker nella scena
	- Stimare la posa dei marker rispetto alla terna camera
	- Stimare la posa della camera rispetto alla terna base del robot
	- Effettuare una trasformazione di coordinate per esprimere la posa dei marker in terna base
	- Stampare la posa dei marker rispetto alla base del robot
	- Visualizzare su Rviz e su gazebo la terna dei marker

### Strumenti e librerie da utilizzare:
Nodi, Topics, Services, Actions, Launch files, TF2ROS OpenCV

## Creatori del Progetto:
- Alessandro Di Filippo
- Dario Pattumelli (forza Roma)
- Annamaria Naddeo
- Lorenzo Soricone
- Claudia Lara Cordova