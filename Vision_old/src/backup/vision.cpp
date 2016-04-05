/*--------------------------------------------------------------------

******************************************************************************
  * @file       vision.cpp
  * @author     Claudio Vilao - ROBOFEI-HT - FEI
  * @version    V0.0.1
  * @created    24/04/2014
  * @Modified   18/07/2014
  * @e-mail
  * @brief      Vision
  ****************************************************************************
/--------------------------------------------------------------------*/

#include"vision.h"

#define INI_FILE_PATH       "../../Control/Data/config.ini"


int main(int argc, char **argv)
{

system("pwd");
	minIni* ini;
	ini = new minIni(INI_FILE_PATH);


	if((pos_servo2=ini->getd("Offset","ID_19",-1024))==-1024){
		cout<<"Erro na leitura do conf.ini";
		return(0);
	}

	if((pos_servo1=ini->getd("Offset","ID_20",-1024))==-1024){
		cout<<"Erro na leitura do conf.ini";
		return(0);
	}


    CvSeq* lines = 0;
    int cont = 0;
    bool head_move2 = false;
    bool head_move1 = false;
    int i;
    int baudnum = DEFAULT_BAUDNUM; //velocidade de transmissao
    int index = 0;
    int deviceIndex = 0; //endereça a USB
    int Moving, PresentPos;
    unsigned int lost_ball = 0; // Conta quantos frames a bola está perdida
    int saida = 0;
    int CommStatus;
    bool gui = false;

    using_shared_memory();

//*********************************************************
//-------------para entrada de argumentos-----------------------------------
 namespace po=boost::program_options;

  po::options_description desc("options");
  desc.add_options()
    ("help", "produce help message")
    ("vb", "apresenta o video da bola na tela")
    ("vg", "apresenta o video do gol na tela")
    ("vl", "apresenta o video e o histograma da localização na tela")
    //("baud,b", po::value<int32_t>(&baudnum),"baud rate")
    ("tg", "abre o trackbars para calibrar a detecção do gol")
    ("tb", "abre o trackbars para calibrar a detecção da bola")
    ("sh", "Abre captura para salvar o histograma")
    ("b", "Escolhe para iniciar com a bola")
    ("g", "Escolhe para iniciar com o gol")
    ("l", "Escolhe para iniciar com a localização")
    ("ws", "Abre sem servo dynamixel")

    ("Far", "Capture open for histogram save from far (Use together with F1...F4 Argments)")
    ("Med", "Capture open for histogram save from medium (Use together with F1...F4 Argments)")
    ("Near", "Capture open for histogram save from Near (Use together with F1...F4 Argments)")

    ("F1", "Choose Soccer Field number 1 (Use together with Far or Med or Near Argments)")
    ("F2", "Choose Soccer Field number 2 (Use together with Far or Med or Near Argments)")
    ("F3", "Choose Soccer Field number 3 (Use together with Far or Med or Near Argments)")
    ("F4", "Choose Soccer Field number 4 (Use together with Far or Med or Near Argments)")

    ("dev0", "Abre Camera 0")
    ("dev1", "Abre Camera 1")
;
  
  po::variables_map variables;
  po::store(po::parse_command_line(argc, argv, desc), variables);
  po::notify(variables); 

	if (variables.count("Far")) 
		{
		    D = "Far";
		}

	if (variables.count("Med")) 
		{
		    D = "Med";
		}

	if (variables.count("Near")) 
		{
		    D = "Near";
		}

	if (variables.count("F1")) 
		{
		    N = "One";
		}

	if (variables.count("F2")) 
		{
		    N = "Two";
		}
	if (variables.count("F3")) 
		{
		    N = "Three";
		}
	if (variables.count("F4")) 
		{
		    N = "Four";
		}


//------------------------------

if (variables.count("help")) 
{
    cout << desc << "\n";
    return 1;
}
/*
if (variables.count("vb"))
	cout<<"Chamou o video da bola\n";
if (variables.count("vg"))
	cout<<"Chamou o video do gol\n";
if (variables.count("vl"))
	cout<<"Chamou o video e o histograma da localização\n";
*/

//--------------------------------------------------------------------------
//*************************************************

  //system("echo | sudo chmod 777 /dev/ttyUSB*");//libera USB

// ---- Open USBDynamixel -----------------------------------------------{
if (!variables.count("ws"))
{
	bool servoComunica = false;

	while(deviceIndex<3)// laço que percorre o servo 0, 1 e 2.
	{
		if( dxl_initialize(deviceIndex, baudnum) == 0 )
		{
			printf( "Failed to open servo%d!\n", deviceIndex );
			if(deviceIndex==2)  // Não encontrou nenhum
			{
				if(servoComunica)
				    printf("Conectou-se a uma placa mas não conseguiu se comunicar com o servo\n");
				else
				    printf("Não encontrou nenhuma placa do servo conectada a porta USB\n");
			    return 0;
			}
			deviceIndex++;      // Não conecta na placa do servo e tenta a proxima porta.
		}
		else
		{
			servoComunica = true;
			printf( "Succeed to open Servo%d!\n", deviceIndex );
    			if(dxl_read_byte( HEAD_TILT, 3 ) == HEAD_TILT)
			{
       			 	printf("Servo%d okay - Connected and communicated!\n", deviceIndex);
			 	break;
			}
    			else
    			{
				printf("Servo wrong or not communicated!\n");
				if(deviceIndex==2)
				{
					printf("Conectou-se a uma placa mas não conseguiu se comunicar com o servo\n");
					return 0;
				}
				deviceIndex++;
			}
		}
	}
}
//-----------------------------------------------------------------------------}

    CvFont font;
    double hScale=1.0;
    double vScale=1.0;
    int    lineWidth=1;
    double posX = 1;
    double posY = 1;
    float raio_bola = 0;
    bool inicio=1;
    float alpha;
    float dist;
    bool print_tela = 0;

   cont_BallSearch = 0;

	BufferBallServo1 = dxl_read_word( HEAD_TILT, P_PRESENT_POSITION_L); 
	BufferBallServo2 = dxl_read_word( HEAD_PAN, P_PRESENT_POSITION_L); 
	BufferGoalServo1 = dxl_read_word( HEAD_TILT, P_PRESENT_POSITION_L); 
	BufferGoalServo2 = dxl_read_word( HEAD_PAN, P_PRESENT_POSITION_L); 
        dxl_write_word(HEAD_TILT, 34, 512); // Usando apenas 50% do torque
        dxl_write_word(HEAD_PAN, 34, 512); // Usando apenas 50% do torque

cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);

	// Tamanho Padrão de captura WebCam Robo- 640x480
	//CvSize tamanho = cvSize(RESOLUCAO_X,RESOLUCAO_Y); //320 240

    	// Abre o dispositivo de captura "0" que é /dev/video0
	int dev = 0;
	if (variables.count("dev0")) dev = 0;
	if (variables.count("dev1")) dev = 1;

 	captura = cvCaptureFromCAM( dev );

		if( !captura )
		{
	        	fprintf( stderr, "ERRO Não há captura na Camera %i/n",dev  );
	        	getchar();
	        	return -1;
        }

cvSetCaptureProperty( captura, CV_CAP_PROP_FRAME_WIDTH, RESOLUCAO_X); //1280   1920
cvSetCaptureProperty( captura, CV_CAP_PROP_FRAME_HEIGHT, RESOLUCAO_Y ); //720   1080

CvSize tamanho = cvSize(cvGetCaptureProperty(captura, CV_CAP_PROP_FRAME_WIDTH),cvGetCaptureProperty(captura, CV_CAP_PROP_FRAME_HEIGHT));

	//Cria a janela do video da bola
	if (variables.count("vb")) cvNamedWindow( "Video Bola", CV_WINDOW_AUTOSIZE );

	// Cria uma janela onde as imagens capturadas serão apresentadas
	if (variables.count("vg")) cvNamedWindow( "Imagem", CV_WINDOW_AUTOSIZE );


        //CvScalar minG = cvScalar(20, 84, 130, 0);
        //CvScalar maxG = cvScalar(270, 256, 255, 0);

//r 80~110
//g 130~150
//b 40~70

			//============Haar ======================================
			double a=0,raio_medio=0, Raio, Distancia;
			IplImage  *frame;
			int       key;
			const char      *filename = "/home/fei/RoboFEI-HT/Vision/src/Ball.xml"; //Name of the classifier

			cascade = ( CvHaarClassifierCascade* )cvLoad( filename, 0, 0, 0 );
			//if(cascade) cout<<"cascade correto"<<endl;
			storage = cvCreateMemStorage(0);
			//if(storage) cout<<"storage correto"<<endl;

			assert( cascade && storage && captura );
			//=======================================================

    usleep(1000000);

//*****************************************************************************

	IplImage*  Quadro_hsv  = cvCreateImage(tamanho, IPL_DEPTH_8U, 3);
	IplImage*  segmentada  = cvCreateImage(tamanho, IPL_DEPTH_8U, 1);

DECISION_ACTION_VISION = 0;

//***********************************************************************************
	if (variables.count("b")) DECISION_ACTION_VISION = 0; // Detecta a Bola.				   //
	if (variables.count("g")) DECISION_ACTION_VISION = 1; // Acha o Gol.				   //
	if (variables.count("l")) DECISION_ACTION_VISION = 2; // Localiza Histograma igual ao armazenado.	   //
	if (variables.count("sh")) DECISION_ACTION_VISION = 3; // Salva o Histograma.			   //
//***********************************************************************************

VISION_STATE =0;

//VideoWriter video("/home/fei/RoboFEI-HT/genfiles/SavedVideo/Bola.avi",CV_FOURCC('M','J','P','G'), 10, Size(RESOLUCAO_X,RESOLUCAO_Y),true);

    while( 1 )
 	{

//*********************************************************************
// 		DECISION_ACTION_VISION = 0 BALL SEARCH
//*********************************************************************

		if(DECISION_ACTION_VISION==0 && IMU_STATE==0) //Decide ver a bola se o robo estiver em pé
		{
	
			frame = cvQueryFrame( captura );
			Raio=detect(frame, posX, posY);
			VISION_MOTOR1_ANGLE = dxl_read_word( HEAD_TILT, P_PRESENT_POSITION_L);
			VISION_MOTOR2_ANGLE = dxl_read_word( HEAD_PAN, P_PRESENT_POSITION_L);
	
			if (Raio>0.1 && Raio<RESOLUCAO_X)
			{
				lost_ball=0;
				cont_BallSearch=0;

				if(a<20)
				{
				raio_medio=raio_medio+Raio;	
				a++;
				}

			else
				{
				a=0;
				raio_medio=raio_medio/10;
				VISION_DIST_BALL=331.59-53.3*log(raio_medio-50);
				//printf("Raio Medio: %g\n", raio_medio);
				//printf("Distancia: %g\n", VISION_DIST_BALL);
				//printf("Motor 1: %d\n", VISION_MOTOR1_ANGLE);
				raio_medio=0;
				}
			}

					if(lost_ball>10) //verifica se depois de 30 frames a bola está perdida no campo
					{
						VISION_SEARCH_BALL = 1;
						BallSearch(inicio);//Procura a bola pelo campo

					//	if(VISION_STATE==0)
					//	      {
					//BallSearch(inicio);//Procura a bola pelo campo
					//	      }
						saida = 0;
						cvPutText (frame, "Procurando robo" ,cvPoint(150,450), &font, cvScalar(255,255,0));
						//call_search = 1;
						inicio = 0;
					}
					else
					{
						VISION_SEARCH_BALL = 0;
						
						if(inicio==0)// Faz o robô parar a cabeça no instante que achou a bola que estava perdida
						{            // Sem esse comando o código não funciona porque ele não para a cabeça
						dxl_write_word(HEAD_TILT, P_GOAL_POSITION_L, dxl_read_word( HEAD_TILT, P_PRESENT_POSITION_L));
						dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, dxl_read_word( HEAD_PAN, P_PRESENT_POSITION_L));
						}

						inicio =1;
						if (HeadFollow(posX, posY, &head_move1, &head_move2) == 1)
						{// Move a cabeça para seguir a bola
						         saida++;
						}

					}
					if(lost_ball<100)
						lost_ball++;

					if (variables.count("vb"))
					{
	 					  //Mat MatFrame(frame);
       	   					 // video.write(MatFrame);
     						  cvShowImage( "Video Bola", frame );
					}

					if( (cvWaitKey(10) & 255) == 27 || saida > 22 || cont_BallSearch > 9 )
					{

						if (cont_BallSearch > 9)
							{
								std::cout << "bola não encontrada" << std::endl;
								VISION_LOST_BALL = 1;
								BufferBallServo1 = VISION_MOTOR1_ANGLE; //Guarda a posição do servo1
								BufferBallServo2 = VISION_MOTOR2_ANGLE; //Guarda a posição do servo2
							}
								else
								{
								VISION_LOST_BALL = 0;
								VISION_DIST_GOAL=0; //zera a variavel do gol
								VISION_MOTOR2_ANGLE = dxl_read_word( HEAD_PAN, P_PRESENT_POSITION_L);
								//std::cout <<"Servo2 Bola = "<< VISION_MOTOR2_ANGLE << std::endl;
								VISION_MOTOR1_ANGLE = dxl_read_word( HEAD_TILT, P_PRESENT_POSITION_L);
								//std::cout <<"Servo1 Bola= "<<VISION_MOTOR1_ANGLE << std::endl;
					   			}	
					}
					BufferBallServo1 = VISION_MOTOR1_ANGLE; //Guarda a posição do servo1
					BufferBallServo2 = VISION_MOTOR2_ANGLE; //Guarda a posição do servo2

					
				 	if( (cvWaitKey(10) & 255) == 27)
				  	{
						cvReleaseImage(&frame);
						cvReleaseCapture(&captura);
						cvDestroyWindow("video");
						cvReleaseHaarClassifierCascade(&cascade);
						cvReleaseMemStorage(&storage);
						break;
					}

	}

} //while(1)

return 0;
}



//=======================================================================================================================
//------------- Função que faz a cabeça centralizar a bola no vídeo------------------------------------------------------
int HeadFollow(float posx, float posy, bool *head_move1, bool *head_move2)
{
     int pan = 0;
    int tilt = 0;
// Posição inicial da cabeça {304, 594} //01 , 02, cabeça

    dxl_write_word(HEAD_TILT, MOVING_SPEED, 300);//300
    dxl_write_word(HEAD_PAN, MOVING_SPEED, 300);//300

//------ Realiza o movimento do Pan -----------------------------------------------------------
    //------ Segue a bola para a esquerda do video -----------------------------------------
    if(posx<(RESOLUCAO_X/2)*(1-CENTERBALL) && *head_move2==false)
    {
        dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, dxl_read_word( HEAD_PAN, P_PRESENT_POSITION_L)+( ((RESOLUCAO_X/2)-posx) * AJUSTE));

        //head_move = true;
    }

    //------ Segue a bola para a direita do video -----------------------------------------
    if(posx>(RESOLUCAO_X/2)*(CENTERBALL+1) && *head_move2==false)
    {
        dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, dxl_read_word( HEAD_PAN, P_PRESENT_POSITION_L)-( (posx-(RESOLUCAO_X/2)) * AJUSTE));

        //dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, dxl_read_word( HEAD_PAN, P_PRESENT_POSITION_L)-(((RESOLUCAO_X/2)-posx) * 0.30) );
        //head_move = true;
    }

    // Para a cabeça se chegou na posição desejada ----------------------------------------
    if(posx>=(RESOLUCAO_X/2)*(1-CENTERBALL) && posx<=(RESOLUCAO_X/2)*(CENTERBALL+1)){
        dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, dxl_read_word( HEAD_PAN, P_PRESENT_POSITION_L));
        pan = 1;

    }

    if(dxl_read_byte( HEAD_PAN, P_MOVING ))
        *head_move2 = true;  // verifica se a cabeça está em movimento
    else
        *head_move2 = false; // verifica se a cabeça está em movimento
//---------------------------------------------------------------------------------------------


//------ Realiza o movimento do Tilt -----------------------------------------------------------
    //------ Segue a bola para a cima do video -----------------------------------------
    if(posy<(RESOLUCAO_Y/2)*(1-CENTERBALL*(RESOLUCAO_X / RESOLUCAO_Y)) && *head_move1==false) // (RESOLUCAO_X / RESOLUCAO_Y) relação entre o tamanho da resolução
    {
		cout<<(dxl_read_word( HEAD_TILT, P_PRESENT_POSITION_L)-( ((RESOLUCAO_Y/2)-posy) * AJUSTE))-pos_servo1<<endl;
        if ((dxl_read_word( HEAD_TILT, P_PRESENT_POSITION_L)-( ((RESOLUCAO_Y/2)-posy) * AJUSTE))>pos_servo1+75)
		dxl_write_word(HEAD_TILT, P_GOAL_POSITION_L, dxl_read_word( HEAD_TILT, P_PRESENT_POSITION_L)-( ((RESOLUCAO_Y/2)-posy) * AJUSTE));
		else
		dxl_write_word(HEAD_TILT, P_GOAL_POSITION_L, pos_servo1+75);

        //head_move = true;
    }

    //------ Segue a bola para a baixo do video -----------------------------------------
    if(posy>(RESOLUCAO_Y/2)*(CENTERBALL*(RESOLUCAO_X / RESOLUCAO_Y)+1) && *head_move1==false) // (RESOLUCAO_X / RESOLUCAO_Y) relação entre o tamanho da resolução 
    {
       dxl_write_word(HEAD_TILT, P_GOAL_POSITION_L, dxl_read_word( HEAD_TILT, P_PRESENT_POSITION_L)+( (posy-(RESOLUCAO_Y/2)) * AJUSTE));

        //dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, dxl_read_word( HEAD_PAN, P_PRESENT_POSITION_L)-(((RESOLUCAO_X/2)-posx) * 0.30) );
        //head_move = true;
    }

    // Para a cabeça se chegou na posição desejada ----------------------------------------
    if(posy>=(RESOLUCAO_Y/2)*(1-CENTERBALL*(RESOLUCAO_X / RESOLUCAO_Y)) && posy<=(RESOLUCAO_Y/2)*(CENTERBALL*(RESOLUCAO_X / RESOLUCAO_Y)+1)){
        dxl_write_word(HEAD_TILT, P_GOAL_POSITION_L, dxl_read_word( HEAD_TILT, P_PRESENT_POSITION_L));

        tilt = 1;
    }

    if(dxl_read_byte( HEAD_TILT, P_MOVING ))
        *head_move1 = true;  // verifica se a cabeça está em movimento
    else
        *head_move1 = false; // verifica se a cabeça está em movimento
//---------------------------------------------------------------------------------------------
if (pan == 1 && tilt == 1)
return 1;
else
return 0;

}
//=======================================================================================================================


void BallSearch(bool inicio)
{
    static unsigned int varredura=0;

    dxl_write_word(HEAD_PAN, MOVING_SPEED, 100);//Seta as velocidades da cabeça
    dxl_write_word(HEAD_TILT, MOVING_SPEED, 200);

    if(inicio)
        varredura--; // continua a varredura de onde parou

    if(varredura > 8 || varredura < 1)
        varredura = 1;

    if(dxl_read_byte(HEAD_PAN, P_MOVING) == 0){
     varredura++;
     cont_BallSearch++;
    }

    if(dxl_read_byte( HEAD_PAN, P_MOVING ) == 0 && dxl_read_byte( HEAD_TILT, P_MOVING ) == 0 && varredura == 8)
    {
        dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, pos_servo2+450);
        dxl_write_word(HEAD_TILT, P_GOAL_POSITION_L, pos_servo1+200);
    }

    if(dxl_read_byte( HEAD_PAN, P_MOVING )==0 && dxl_read_byte( HEAD_TILT, P_MOVING ) == 0 &&varredura==7)
    {
        dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, pos_servo2-440);
        dxl_write_word(HEAD_TILT, P_GOAL_POSITION_L, pos_servo1+200);
    }


    if(dxl_read_byte( HEAD_PAN, P_MOVING )==0 && dxl_read_byte( HEAD_TILT, P_MOVING ) == 0 && varredura==6)
    {
        dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, pos_servo2+450);
        dxl_write_word(HEAD_TILT, P_GOAL_POSITION_L, pos_servo1+100);
    }


    if(dxl_read_byte( HEAD_PAN, P_MOVING )==0 && dxl_read_byte( HEAD_TILT, P_MOVING ) == 0 && varredura==5)
    {
        dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, pos_servo2 -440);
        dxl_write_word(HEAD_TILT, P_GOAL_POSITION_L, pos_servo1+100);
    }


    if(dxl_read_byte( HEAD_PAN, P_MOVING )==0 && dxl_read_byte( HEAD_TILT, P_MOVING ) == 0 && varredura==4)
    {
        dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, pos_servo2+450);
        dxl_write_word(HEAD_TILT, P_GOAL_POSITION_L, pos_servo1+200);
    }


    if(dxl_read_byte( HEAD_PAN, P_MOVING )==0 && dxl_read_byte( HEAD_TILT, P_MOVING ) == 0 && varredura==3)
    {
        dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, pos_servo2-440);
        dxl_write_word(HEAD_TILT, P_GOAL_POSITION_L, pos_servo1+200);
    }

    if(dxl_read_byte( HEAD_TILT, P_MOVING )==0 && dxl_read_byte( HEAD_TILT, P_MOVING ) == 0 && varredura==2)
    {
        dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, pos_servo2-440);
        dxl_write_word(HEAD_TILT, P_GOAL_POSITION_L, pos_servo1+200);

    }

    if(dxl_read_byte( HEAD_TILT, P_MOVING )==0 && dxl_read_byte( HEAD_TILT, P_MOVING ) == 0 && varredura==1)
    {
        dxl_write_word(HEAD_PAN, MOVING_SPEED, 700);
        dxl_write_word(HEAD_TILT, MOVING_SPEED, 700);
        dxl_write_word(HEAD_PAN, P_GOAL_POSITION_L, pos_servo2);
        dxl_write_word(HEAD_TILT, P_GOAL_POSITION_L, pos_servo1);
    }

}


double detect(IplImage *Quadro, double &posx, double &posy)
{
			CvSize tamanho = cvSize(cvGetCaptureProperty(captura, CV_CAP_PROP_FRAME_WIDTH),cvGetCaptureProperty(captura, CV_CAP_PROP_FRAME_HEIGHT));

			IplImage*  Quadro_hsv  = cvCreateImage(tamanho, IPL_DEPTH_8U, 3);
			IplImage*  segmentada  = cvCreateImage(tamanho, IPL_DEPTH_8U, 1);

			//CvScalar minB = cvScalar(0, 171, 100);//4 160 //160
			//CvScalar maxB = cvScalar(25, 255, 255);//11 //250
	if(LOCALIZATION_FIND_ROBOT == 1)
            {
                //Red:
			    CvScalar minB = cvScalar(160, 68, 32);//4 160 //160
			    CvScalar maxB = cvScalar(179, 255, 255);//11 //250

			    //Para HSV	        
			    cvCvtColor(Quadro, Quadro_hsv, CV_BGR2HSV);

	            // Filtrar cores que não interessam.
        	    cvInRangeS(Quadro_hsv, minB, maxB, segmentada);

			
		        cvErode(segmentada,segmentada,cvCreateStructuringElementEx(5,5,0,0,CV_SHAPE_ELLIPSE));
		        cvDilate(segmentada,segmentada,cvCreateStructuringElementEx(5,5,0,0,CV_SHAPE_ELLIPSE));

		        std::string str;
		        double area=0;

                CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
                cvMoments(segmentada, moments, 1);

		        Mat img (segmentada);
		    
  		        cvShowImage( "Filtragem das cores", segmentada ); // O stream depois da filtragem de cores
		    
		          cv::Point Coord;
		        cv::Moments mm = cv::moments(img,false);
		        double moment10 = mm.m10;
		        double moment01 = mm.m01;
		        double moment00 = mm.m00;
		        Coord.x = int(moment10 / moment00);
		        Coord.y = int(moment01 / moment00);
		        VISION_AREA_SEGMENT = moment00;

			    if (Coord.x<0)
				    Coord.x = 0;
			    if (Coord.y<0)
				    Coord.y = 0;

			    posx=Coord.x;
			    posy=Coord.y;

			    if (posx<=0 || posy<=0)
			    	return (-1);
			    else
			    {
				    cvCircle(Quadro , Coord, 10, CV_RGB(0,255,0), -1, 8, 0 );
			        VISION_DELTA_ORIENT =  pos_servo2 - VISION_MOTOR2_ANGLE; //positivo direita / negativo esquerda
				    std::cout << "delta orientacao " << VISION_DELTA_ORIENT << std::endl;
				    std::cout << "area " << VISION_AREA_SEGMENT << std::endl;
				    return (1);
			    }
			 }
			
			
			
            if(LOCALIZATION_FIND_ROBOT == 4)
            {
                //Blue:
			    CvScalar minB = cvScalar(77, 80, 0);//4 160 //160
			    CvScalar maxB = cvScalar(113, 255, 255);//11 //250

			    //Para HSV	        
			    cvCvtColor(Quadro, Quadro_hsv, CV_BGR2HSV);

	            // Filtrar cores que não interessam.
        	    cvInRangeS(Quadro_hsv, minB, maxB, segmentada);

			
		        cvErode(segmentada,segmentada,cvCreateStructuringElementEx(5,5,0,0,CV_SHAPE_ELLIPSE));
		        cvDilate(segmentada,segmentada,cvCreateStructuringElementEx(7,7,0,0,CV_SHAPE_ELLIPSE));

		        std::string str;
		        double area=0;

                CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
                cvMoments(segmentada, moments, 1);

		        Mat img (segmentada);
		    
  		        cvShowImage( "Filtragem das cores", segmentada ); // O stream depois da filtragem de cores
		    
		          cv::Point Coord;
		        cv::Moments mm = cv::moments(img,false);
		        double moment10 = mm.m10;
		        double moment01 = mm.m01;
		        double moment00 = mm.m00;
		        Coord.x = int(moment10 / moment00);
		        Coord.y = int(moment01 / moment00);
		        VISION_AREA_SEGMENT = moment00;

			    if (Coord.x<0)
				    Coord.x = 0;
			    if (Coord.y<0)
				    Coord.y = 0;

			    posx=Coord.x;
			    posy=Coord.y;

			    if (posx<=0 || posy<=0)
			    	return (-1);
			    else
			    {
				    cvCircle(Quadro , Coord, 10, CV_RGB(0,255,0), -1, 8, 0 );
			        VISION_DELTA_ORIENT =  pos_servo2 - VISION_MOTOR2_ANGLE; //positivo direita / negativo esquerda
				    std::cout << "delta orientacao " << VISION_DELTA_ORIENT << std::endl;
				    std::cout << "area " << VISION_AREA_SEGMENT << std::endl;
				    return (1);
			    }
			 }
			 
	if(LOCALIZATION_FIND_ROBOT == 3)
            {
                //Yellow:
			    CvScalar minB = cvScalar(18, 82, 152);//4 160 //160
			    CvScalar maxB = cvScalar(26, 255, 255);//11 //250

			    //Para HSV	        
			    cvCvtColor(Quadro, Quadro_hsv, CV_BGR2HSV);

	            // Filtrar cores que não interessam.
        	    cvInRangeS(Quadro_hsv, minB, maxB, segmentada);

			
		        cvErode(segmentada,segmentada,cvCreateStructuringElementEx(5,5,0,0,CV_SHAPE_ELLIPSE));
		        cvDilate(segmentada,segmentada,cvCreateStructuringElementEx(7,7,0,0,CV_SHAPE_ELLIPSE));

		        std::string str;
		        double area=0;

                CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
                cvMoments(segmentada, moments, 1);

		        Mat img (segmentada);
		    
  		        cvShowImage( "Filtragem das cores", segmentada ); // O stream depois da filtragem de cores
		    
		          cv::Point Coord;
		        cv::Moments mm = cv::moments(img,false);
		        double moment10 = mm.m10;
		        double moment01 = mm.m01;
		        double moment00 = mm.m00;
		        Coord.x = int(moment10 / moment00);
		        Coord.y = int(moment01 / moment00);
		        VISION_AREA_SEGMENT = moment00;

			    if (Coord.x<0)
				    Coord.x = 0;
			    if (Coord.y<0)
				    Coord.y = 0;

			    posx=Coord.x;
			    posy=Coord.y;

			    if (posx<=0 || posy<=0)
			    	return (-1);
			    else
			    {
				    cvCircle(Quadro , Coord, 10, CV_RGB(0,255,0), -1, 8, 0 );
			        VISION_DELTA_ORIENT =  pos_servo2 - VISION_MOTOR2_ANGLE; //positivo direita / negativo esquerda
				    std::cout << "delta orientacao " << VISION_DELTA_ORIENT << std::endl;
				    std::cout << "area " << VISION_AREA_SEGMENT << std::endl;
				    return (1);
			    }
			 }
	//localiza bola laranja!!!
	   if(LOCALIZATION_FIND_ROBOT == 2)
            {
                //Orange:
			    CvScalar minB = cvScalar(2, 179, 83);//4 160 //160
			    CvScalar maxB = cvScalar(8, 247, 211);//11 //250

			    //Para HSV	        
			    cvCvtColor(Quadro, Quadro_hsv, CV_BGR2HSV);

	            // Filtrar cores que não interessam.
        	    cvInRangeS(Quadro_hsv, minB, maxB, segmentada);

			
		        cvErode(segmentada,segmentada,cvCreateStructuringElementEx(5,5,0,0,CV_SHAPE_ELLIPSE));
		        cvDilate(segmentada,segmentada,cvCreateStructuringElementEx(7,7,0,0,CV_SHAPE_ELLIPSE));

		        std::string str;
		        double area=0;

                CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
                cvMoments(segmentada, moments, 1);

		        Mat img (segmentada);
		    
  		        cvShowImage( "Filtragem das cores", segmentada ); // O stream depois da filtragem de cores
		    
		          cv::Point Coord;
		        cv::Moments mm = cv::moments(img,false);
		        double moment10 = mm.m10;
		        double moment01 = mm.m01;
		        double moment00 = mm.m00;
		        Coord.x = int(moment10 / moment00);
		        Coord.y = int(moment01 / moment00);
		        VISION_AREA_SEGMENT = moment00;

			    if (Coord.x<0)
				    Coord.x = 0;
			    if (Coord.y<0)
				    Coord.y = 0;

			    posx=Coord.x;
			    posy=Coord.y;

			    if (posx<=0 || posy<=0)
			    	return (-1);
			    else
			    {
				    cvCircle(Quadro , Coord, 10, CV_RGB(0,255,0), -1, 8, 0 );
			        VISION_DELTA_ORIENT =  pos_servo2 - VISION_MOTOR2_ANGLE; //positivo direita / negativo esquerda
				    std::cout << "delta orientacao " << VISION_DELTA_ORIENT << std::endl;
				    std::cout << "area " << VISION_AREA_SEGMENT << std::endl;
				    return (1);
			    }
			 }
			
			

}






