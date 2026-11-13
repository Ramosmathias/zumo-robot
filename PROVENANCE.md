# Provenance du projet Zumo

## Sources

Le premier sketch et le classeur `graphiques.xlsx` proviennent de l’archive de remise du binôme. Le deuxième sketch part de la version de travail `RAMOS_TASSAIN_ZUMO_15.ino` du 12 avril 2025, retrouvée dans le carnet de croquis Arduino. Les modifications ont été faites dans les copies du dépôt.

| Élément | Origine constatée |
|---|---|
| `gyro_use.h` et `gyro_use.cpp` | Fichiers des exemples `calibrage1`, `essai1` et `navi_pilote_obst` de `ZUMO_Project.zip`. Ils ont servi à la compilation des versions historiques et sont conservés dans le dossier local de test. |
| Rotations et étalonnage des capteurs de ligne | Les routines reprises du cours ont été remplacées par `ZumoMotion`, commun aux deux programmes. La nouvelle version conserve la séquence de balayage à 90°, −180° et 90° ainsi que le facteur effectif du calcul du cap de la version du binôme. Une rotation s’arrête après six secondes si l’angle demandé reste hors d’atteinte. Le comportement reste à vérifier sur le robot. |
| Commandes de calibration, relevés d’encodeurs et navigation | Conservés depuis les deux versions du binôme. La répartition des contributions individuelles reste à confirmer. |
| Arduino et bibliothèque Zumo32U4 | Dépendances externes. [Bibliothèque Pololu](https://github.com/pololu/zumo-32u4-arduino-library) |

Le code d’origine appliquait un coefficient équivalent à 0,0175 °/s par pas. Le mode de rotation configuré par Pololu indique 0,07 °/s par pas dans ses exemples. La copie conserve le calcul historique en attendant de pouvoir mesurer l’angle réel sur le robot.

## Données

Les CSV reprennent les trois onglets du classeur de remise. Les points à zéro présents dans le classeur sont écartés des séries d’essais et des ajustements, qui portent sur les cinq réglages moteur et les distances de 5 à 20 cm. Les vitesses et les ajustements sont recalculés à partir des valeurs exportées. Le graphique de vitesse côté droit utilise la colonne des commandes droites pour la moyenne. Dans le graphique d’origine, cette série référençait la colonne des commandes gauches.
