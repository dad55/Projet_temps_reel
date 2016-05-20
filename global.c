/*
 * File:   global.h
 * Author: pehladik
 *
 * Created on 21 avril 2011, 12:14
 */

#include "global.h"

RT_TASK tServeur;
RT_TASK tconnect;
RT_TASK tmove;
RT_TASK tenvoyer;
//////////////////////////////////
RT_TASK tcamera;
RT_TASK tgestion_wdt;
RT_TASK tsurvBatterie;

RT_MUTEX mutexEtat;
RT_MUTEX mutexMove;
RT_MUTEX mutexCommRobot;
RT_MUTEX mutexMessage;
RT_MUTEX mutexCamera;
RT_MUTEX mutexPosition;

RT_SEM semConnecterRobot;
RT_SEM semWdtRobot;

RT_QUEUE queueMsgGUI;

int etatCommMoniteur = 1;
int etatCommRobot = 1;
int etatCamera = 0;
int etatPosition = 0;
int cptTestComm = 0;
DRobot *robot;
DMovement *move;
DServer *serveur;


int MSG_QUEUE_SIZE = 10;

int PRIORITY_TSERVEUR = 14;
int PRIORITY_TCONNECT = 13;
int PRIORITY_TMOVE = 10;
int PRIORITY_TENVOYER = 13;
int PRIORITY_TCAMERA = 11;
int PRIORITY_TGESTION_WDT = 15;
int PRIORITY_TSURVBATTERIE = 12;
