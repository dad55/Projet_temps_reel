/* 
 * File:   global.h
 * Author: pehladik
 *
 * Created on 12 janvier 2012, 10:11
 */

#ifndef GLOBAL_H
#define	GLOBAL_H

#include "includes.h"

/* @descripteurs des tâches */
extern RT_TASK tServeur;
extern RT_TASK tconnect;
extern RT_TASK tmove;
extern RT_TASK tenvoyer;
extern RT_TASK tcamera;
extern RT_TASK tgestion_wdt;
extern RT_TASK tsurvBatterie;

/* @descripteurs des mutex */
extern RT_MUTEX mutexEtat;
extern RT_MUTEX mutexMove;
extern RT_MUTEX mutexCommRobot;
extern RT_MUTEX mutexMessage;
extern RT_MUTEX mutexCamera;
extern RT_MUTEX mutexPosition;

/* @descripteurs des sempahores */
extern RT_SEM semConnecterRobot;
extern RT_SEM semWdtRobot;

/* @descripteurs des files de messages */
extern RT_QUEUE queueMsgGUI;

/* @variables partagées */
extern int etatCommMoniteur;
extern int etatCommRobot;
extern DServer *serveur;
extern DRobot *robot;
extern DMovement *move;
extern int etatCamera;
extern int etatPosition;
extern int cptTestComm;



/* @constantes */
extern int MSG_QUEUE_SIZE;
extern int PRIORITY_TSERVEUR;
extern int PRIORITY_TCONNECT;
extern int PRIORITY_TMOVE;
extern int PRIORITY_TENVOYER;
extern int PRIORITY_TCAMERA;
extern int PRIORITY_TGESTION_WDT;
extern int PRIORITY_TSURVBATTERIE;



#endif	/* GLOBAL_H */

