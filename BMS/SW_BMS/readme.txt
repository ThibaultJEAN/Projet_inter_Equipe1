************************************************************
*							   *
*		README FILE BMS SOFTWARE		   *
*							   *
************************************************************
author: jeremy aubert for GEI Corp



Utilisation de notre API.


I- Initialisation

Nous proposons une fonction "BMSManagement_Init" permettant
l'initialisation des jauges. Cette initialisation regroupe :
	- la vérification de la bonne connexion entre le 
	microcontroleur et la jauge (avec un mode bloquant
	+ message d'erreur si pas de connexion)

	- l'écriture dans le registre de "control" du mode
	automatique. Cette écriture permet de faire une 
	mise à jour des registres (tension, courant, ...).

Une fonctionnalité n'est tout de fois pas inclu dans cette 
fonction : l'initialisation de la valeur du SOC. D'après
la documentation technique du composant LTC2944, la valeur
par défaut du SOC est de 50%. Pour le moment, il faut 
utiliser la fonction setSOC présente dans InfoBatt.c
Les paramètres à utiliser sont :
	- I2C_HandleTypeDef nb_batt : il faut utiliser
	les constantes BATTERY1 et BATTERY2.
	- uint16_t initValue : il faut entrer la valeur
	en hexadecimal selon le taux de charge de la 
	batterie. Cette valeur se fixe selon la formule
	suivante :
	     (initValue)_16=deltaQ*StateOfCharge



II- La lecture des informations se fait par la fonction 
BMSManagement_getInfo. Les paramètres sont :
	- char info : 'V' pour avoir la tension, 'I' pour
	avoir le courant, 'S' pour avoir le SOC.
	- I2C_HandleTypeDef nb_batt : il faut utiliser
	les constantes BATTERY1 et BATTERY2.

III- Fonction support

saveSOC - Afin de sauvegarder la dernière valeur de SOC
des batteries pour la sauvegarder dans une mémoire.

checkLink - A utiliser si vous souhaitez vérifier le lien
à tout moment entre le µC et la jauge.