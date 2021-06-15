static int Richtungsgeber = 0;
static float WirdUmbenannt = 0;

if (counter == 0){                      //Ist für den Anfang damit der Turtlebot von der mitte aus zur ersten Wand fährt

    if(averageVorne <= 0.2){            //Turtlebot hat Wand vor sich erreicht

        if(averageLinks <= 0.2){            //Um Wand auf der Rechten Seite des Turtlebots zu haben muss er sich Linksdrehen
            switch  (Richtungsgeber){
                case 0:
                        while(WirdUmbenannt < 90){
                            robot_move(NEUNZIG_LINKS);
                        }
                        Richtungsgeber = 90;
                        break;
                case 90:
                        while(WirdUmbenannt < 180){
                            robot_move(NEUNZIG_LINKS);
                        }
                        Richtungsgeber = 180;
                        break;
                case 0:
                        while(WirdUmbenannt < 90){
                            robot_move(NEUNZIG_LINKS);
                        }
                        Richtungsgeber = 90;
break;
case 90:
while(WirdUmbenannt < 180){
robot_move(NEUNZIG_LINKS);
}
Richtungsgeber = 180;
break;
            }                          //180° Linksdrehung, da Wand links vorhanden ist
            counter = 1;                    //um in if-Bedingung (counter ==1) zu kommen
        } else{
            robot_move(NEUNZIG_LINKS);          //90° Linksdrehung
            counter = 1;
            }
    } else{
        robot_move(GERADEAUS_LANG);                              //Turtlebot fährt dicht zur ersten Wand ab der er sich orientieren kann
        }
} else{                             //ab hier an kann der Turtlebot sich orientieren
    ros::spinOnce();

    if(averageRechts <= 0.2){            //Turtlebot hat rechts neben sich eine Wand und kann somit den Rechte-Hand Algorythmus durchführen
        if(averageVorne <= 0.2){             //Turtlebot hat eine Wand vor sich und eine Wand rechts neben sich
            if(averageLinks <= 0.2){             //Kommentar aus Zeile 153 + noch eine Wand dicht auf der Linken Seite
                robot_move(HUNDERTACHTZIG);                                    //180° Linksdrehung da der Turtlebot in einer Sackgasse ist
            } else{                              //Links hat der Turtlebot keine Wand
                robot_move(NEUNZIG_LINKS);                                 //Linksdrehung um 90°
                }
        } else{                              //Turtlebot hat keine Wand vor sich aber rechts neben sich, daher kann er geradeaus fahren
            robot_move(GERADEAUS_LANG);
            }
    } else {                             //Turtlebot hat keine Wand rechts neben sich daher ein Gang oder eine Tür
        robot_move(RECHTS_GERADEAUS_KURZ);                                   //Turtlebot dreht sich 90° nach rechts und bewegt sich ein Stück nach vorne
        }
}