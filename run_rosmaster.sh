#!/bin/bash
# Affiche nouvelle version
echo "Démarrage nouvelle version (Ctrl+C pour annuler...)"
sleep 3
# Kill l'ancien rosmaster
echo "Kill de l'ancienne instance..."
# pkill -f rosmaster &
# Obtenir IP machine
IP_ADDRESS=$(hostname -I | awk '{print $1}')
# Construire lURL en utilisant ladresse IP
ROS_MASTER_URI="http://$IP_ADDRESS:11311/"
ROS_IP=$IP_ADDRESS
# Exporter les variables
export ROS_MASTER_URI
export ROS_IP
# Afficher les valeurs des variables
echo ROS_MASTER_URI=$ROS_MASTER_URI
echo ROS_IP=$ROS_IP

cd ~/code/kr1bou_proto
catkin_make
source devel/setup.bash

# Boucle while pour vérifier si l'adresse IP est non nulle
while [ -z "$IP_ADDRESS" ]; do
    echo "Aucune adresse IP trouvée. Attente d'une adresse IP valide..."
    sleep 0.5
    IP_ADDRESS=$(hostname -I | awk '{print $1}')
    ROS_MASTER_URI="http://$IP_ADDRESS:11311/"
    ROS_IP=$IP_ADDRESS
    export ROS_MASTER_URI
    export ROS_IP
done

roslaunch kr1bou_launch master.launch &
ps aux | grep master.launch

