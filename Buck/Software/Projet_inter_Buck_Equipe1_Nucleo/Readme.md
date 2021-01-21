Indications de fonctionnement pour le code Buck :

PWM sur TIM1. Fréquence réglable par define. Duty min et max aussi. Valeurs actuelles : 150kHz, 10%, 90%
Acquisitions multiples par ADC1 avec DMA et trigger par TIM2, transfert dans les variables par interruption ADC de conversion complète. F_acq réglable par define, valeur actuelle : 5kHz

Les regulations actuelles sont des proportionnel uniquement. Régulation MPPT non fonctionnelle.

Interface Uart (debug rapide) : 
La carte affiche ses mesures toutes les 500ms.
Elle démarre en mode CV 24V.
Pour changer de mode : 'v' pour CV, 'c' pour CC, 'm' pour MPPT, 'o' pour OpenLoop
Pour changer le setpoint : '+' ou '-'. Incrément varie en fonction des modes : CV +-1V, CC +-250mA, OL +-10%.
Attention, ne pas envoyer plusieurs commandes trop rapidement, sinon le buffer panique et il faut reset la carte.


Attention, si la GUI CubeMX est utilisée pour regénérer le projet, il faut ensuite commenter les lignes 
"htim1.Init.Period = 65535;" et "//htim2.Init.Period = 999;".