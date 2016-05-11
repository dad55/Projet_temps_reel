#include "fonctions.h"

int write_in_queue(RT_QUEUE *msgQueue, void * data, int size);

void envoyer(void * arg) {
    DMessage *msg;
    int err;

    while (1) {
        rt_printf("tenvoyer : Attente d'un message\n");
        if ((err = rt_queue_read(&queueMsgGUI, &msg, sizeof (DMessage), TM_INFINITE)) >= 0) {
            rt_printf("tenvoyer : envoi d'un message au moniteur\n");
            serveur->send(serveur, msg);
            msg->free(msg);
        } else {
            rt_printf("Error msg queue write: %s\n", strerror(-err));
        }
    }
}

void connecter(void * arg) {
    int status;
    DMessage *message;

    rt_printf("tconnect : Debut de l'exécution de tconnect\n");

    while (1) {
        rt_printf("tconnect : Attente du sémarphore semConnecterRobot\n");
        rt_sem_p(&semConnecterRobot, TM_INFINITE);
        rt_printf("tconnect : Ouverture de la communication avec le robot\n");
	rt_mutex_acquire(&mutexCommRobot, TM_INFINITE);
        status = robot->open_device(robot);
	rt_mutex_release(&mutexCommRobot);

        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        etatCommRobot = status;
        rt_mutex_release(&mutexEtat);

        if (status == STATUS_OK) {
		rt_mutex_acquire(&mutexCommRobot, TM_INFINITE);
            status = robot->start_insecurely(robot);
		rt_mutex_release(&mutexCommRobot);
            if (status == STATUS_OK){
                rt_printf("tconnect : Robot démarrer\n");
            }
        }

        message = d_new_message();
        message->put_state(message, status);

        rt_printf("tconnecter : Envoi message\n");
        message->print(message, 100);

        if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
            message->free(message);
        }
    }
}

void communiquer(void *arg) {
    DMessage *msg = d_new_message();
    int var1 = 1;
    int num_msg = 0;

    rt_printf("tserver : Début de l'exécution de serveur\n");
    serveur->open(serveur, "8000");
    rt_printf("tserver : Connexion\n");

    rt_mutex_acquire(&mutexEtat, TM_INFINITE);
    etatCommMoniteur = 0;
    rt_mutex_release(&mutexEtat);

    while (var1 > 0) {
        rt_printf("tserver : Attente d'un message\n");
        var1 = serveur->receive(serveur, msg);
        num_msg++;
        if (var1 > 0) {
            switch (msg->get_type(msg)) {
                case MESSAGE_TYPE_ACTION:
                    rt_printf("tserver : Le message %d reçu est une action\n",
                            num_msg);
                    DAction *action = d_new_action();
                    action->from_message(action, msg);
                    switch (action->get_order(action)) {
                        case ACTION_CONNECT_ROBOT:
                            rt_printf("tserver : Action connecter robot\n");
                            rt_sem_v(&semConnecterRobot);
                            break;
			case ACTION_FIND_ARENA :
				etatCamera = ACTION_FIND_ARENA ;
				break;
			case ACTION_ARENA_FAILED :
				etatCamera = ACTION_ARENA_FAILED;
				break;
			case ACTION_ARENA_IS_FOUND :  
				etatCamera = ACTION_ARENA_IS_FOUND;
				break;
			case ACTION_COMPUTE_CONTINUOUSLY_POSITION :
				etatPosition = 1;
				break;
			case ACTION_STOP_COMPUTE_POSITION :	
				etatPosition = 0;
				break;
                    }
                    break;
                case MESSAGE_TYPE_MOVEMENT:
                    rt_printf("tserver : Le message reçu %d est un mouvement\n",
                            num_msg);
                    rt_mutex_acquire(&mutexMove, TM_INFINITE);
                    move->from_message(move, msg);
                    move->print(move);
                    rt_mutex_release(&mutexMove);
                    break;
            }
        }
    }
}

void deplacer(void *arg) {
    int status = 1;
    int gauche;
    int droite;
    DMessage *message;

    rt_printf("tmove : Debut de l'éxecution de periodique à 1s\n");
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);

    while (1) {
        /* Attente de l'activation périodique */
        rt_task_wait_period(NULL);
        rt_printf("tmove : Activation périodique\n");

        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        status = etatCommRobot;
        rt_mutex_release(&mutexEtat);

        if (status == STATUS_OK) {
            rt_mutex_acquire(&mutexMove, TM_INFINITE);
            switch (move->get_direction(move)) {
                case DIRECTION_FORWARD:
                    gauche = MOTEUR_ARRIERE_LENT;
                    droite = MOTEUR_ARRIERE_LENT;
                    break;
                case DIRECTION_LEFT:
                    gauche = MOTEUR_ARRIERE_LENT;
                    droite = MOTEUR_AVANT_LENT;
                    break;
                case DIRECTION_RIGHT:
                    gauche = MOTEUR_AVANT_LENT;
                    droite = MOTEUR_ARRIERE_LENT;
                    break;
                case DIRECTION_STOP:
                    gauche = MOTEUR_STOP;
                    droite = MOTEUR_STOP;
                    break;
                case DIRECTION_STRAIGHT:
                    gauche = MOTEUR_AVANT_LENT;
                    droite = MOTEUR_AVANT_LENT;
                    break;
            }
            rt_mutex_release(&mutexMove);
		
		rt_mutex_acquire(&mutexCommRobot, TM_INFINITE);
            status = robot->set_motors(robot, gauche, droite);
		rt_mutex_release(&mutexCommRobot);

            if (status != STATUS_OK) {
                rt_mutex_acquire(&mutexEtat, TM_INFINITE);
                etatCommRobot = status;
                rt_mutex_release(&mutexEtat);

                message = d_new_message();
                message->put_state(message, status);

                rt_printf("tmove : Envoi message\n");
                if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
                    message->free(message);
                }
            }
        }
    }
}

int write_in_queue(RT_QUEUE *msgQueue, void * data, int size) {
    void *msg;
    int err;

    msg = rt_queue_alloc(msgQueue, size);
    memcpy(msg, &data, size);

    if ((err = rt_queue_send(msgQueue, msg, sizeof (DMessage), Q_NORMAL)) < 0) {
        rt_printf("Error msg queue send: %s\n", strerror(-err));
    }
    rt_queue_free(&queueMsgGUI, msg);

    return err;
}


void camera(void) {
	DCamera* camera;
	DImage* image;
	DJpegimage* jpeg;
	DMessage* message;
	DArena* arena;
	DPosition* position;

	camera =  d_new_camera();
	image = d_new_image();
	jpeg = d_new_jpegimage();
	arena = d_new_arena();
	position = d_new_position();

	camera->open(camera);
	
	rt_printf("tcamera : Debut de l'éxecution periodique à 600ms\n");
    	rt_task_set_periodic(NULL, TM_NOW, 600000000);

	while(1){
		rt_task_wait_period(NULL);
       		rt_printf("tcamera : Activation périodique\n");
		if(serveur->is_active(serveur)){
			if(etatCamera == ACTION_FIND_ARENA){
				camera->get_frame(camera,image);
				arena = image->compute_arena_position(image);
				d_imageshop_draw_arena(image, arena);
				jpeg->compress(jpeg,image);
				message = d_new_message();
				message->put_jpeg_image(message,jpeg);
				if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) 
		           		message->free(message);
			}
			else{
				camera->get_frame(camera,image);
				if(arena != NULL)
					d_imageshop_draw_arena(image, arena);
				if(etatPosition == 1){
					position = image->compute_robot_position(image, arena);
					d_imageshop_draw_position(image, position);
					if(position != NULL){
						message = d_new_message();
						message->put_position(message,position);
						if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) 
		           				message->free(message);
					}
				}
				jpeg->compress(jpeg,image);
				message = d_new_message();
				message->put_jpeg_image(message,jpeg);
				if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) 
		           		message->free(message);
			}
		}
	}
}

void gestion_wdt(void){
	int status = 1;
	DMessage* message;

	rt_printf("tgestion_wdt : Debut de l'éxecution periodique à 950ms (max 1050ms)\n");
    	rt_task_set_periodic(NULL, TM_NOW, 950000000);

	while(1){
		rt_task_wait_period(NULL);
		rt_printf("tgestion_wdt : Activation périodique\n");
		
		rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        	status = etatCommRobot;
        	rt_mutex_release(&mutexEtat);
		if(status == STATUS_OK){
			rt_mutex_acquire(&mutexCommRobot, TM_INFINITE);
			status = robot->reload_wdt(robot);
			rt_mutex_release(&mutexCommRobot);
			if (status != STATUS_OK) {
				rt_mutex_acquire(&mutexEtat, TM_INFINITE);
				etatCommRobot = status;
				rt_mutex_release(&mutexEtat);

				message = d_new_message();
				message->put_state(message, status);

				rt_printf("tgestion_wdt : Envoi message\n");
				if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
					message->free(message);
				}
            		}
		}
	}
}

