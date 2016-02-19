/* Code part of the project "New task generation for humanoid robots based on case and user communication"
 *
 * Author: Raúl Fernández Fernández(raulfernandezbis@gmail.com)
 *
 * The goal of this node is to be a form where the system gets the information resuered from the user to generate
 * the new task. The way this is done is just by simply a writing question/answer input from the terminal.
 *
 */

#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include <string>
#include <teo_moveit/communication_srv.h>

std::string readkeyboard(){

    std::string input;
    std::cin.clear(); //Clear the buffer
    std::cin>>input;

    return input;
}

bool communication(teo_moveit::communication_srv::Request  &req, teo_moveit::communication_srv::Response &res){

    std::string flag;

    std::cout<<"Greetings! If you dont mind, I am going to ask you a few questions before i can perform the task"<<std::endl;
    //std::cout<<"Te voy a hacer unas cuantas preguntillas antes de que pueda hacer la tarea"<<std::endl;

    //cout<<"Como se llama la nueva tarea que me vas a enseñar"<<endl; //Esta linea en principio ahora no nos es relevante.
    //Aqui tenemos dos opciones trabajar con tareas de alto nivel, ejemplo: el usuario dice que la tarea de cavar es : "coger pala + cavar + dejar pala";
    //o trabajar con tareas de bajo nivel (atómicas) ejemplo: "mover brazo X pala + pick pala + mover brazo X a y + cavar (nueva accion atómica) + ..."
    //Utilizando la primera opción es mucho más natural la conversación, sin embargo tenemos el problema de entender esa tarea a alto nivel. Voy a hacer un combo
    //utilizar ambos niveles.
    std::cout<<"Which is the name of the new task i am going to perform?"<<std::endl;
    //std::cout<<"¿Como se llama la tarea que voy a realizar?"<<std::endl;
    res.high_task=readkeyboard();
    std::cout<<"Where should i be to perform the task? (garden or kitchen)"<<std::endl;
    //Two possibilites kitchen or garden.
    //std::cout<<"¿Donde debería estar para hacer la tarea? (kitchen, garden...)"<<std::endl;
    res.entorno.push_back(readkeyboard());
    
    std::cout<<"Which object/tool am i going to need to perform the task?"<<std::endl;
    //std::cout<<"Que objeto necesito utilizar para llevar acabo la tarea:";
    res.object=readkeyboard(); // This way we know the high level structure of the task [pick "object" + "task" + place "object"]

    //De esta forma ya está definida la tarea de coger y place, posteriormente se pueden añadir parámetros de grasp y todo eso, por loq ue necesitaremos más información del objeto
    std::cout<<"Once i have already picked the object " <<res.object <<". Could you describe me the detailed steps that i should follow to perform the task "<<res.high_task<<"? When finish just write 100"<<std::endl;
    //std::cout<<"Una vez que he cogido el objeto " <<res.object <<" ¿Me podrías decir los pasos detallados que tengo que seguir para llevar acabo la tarea "<<res.high_task<<"? Cuando ya no haya más pasos simplemente escribe fin"<<std::endl;
    std::cout<<"Lets begin for the begining: ";
    //std::cout<<"Empezemos por el principio: ";
    int i=1;
    while (i<20){
        std::cout<<"What action should i execute now? (0=move_group; 1=pick; 2=place)"<<std::endl;
        int num_task;
        std::cin>>num_task;
        res.atomic_task.push_back(num_task);
        if (res.atomic_task.back()== 100)
               break;

        //std::cout<<"¿Donde debería estar para hacer la tarea?"<<std::endl;
        //res.entorno.push_back(readkeyboard());

        //Aqui habría que comprobar si la atomic task que nos dice está definida dentro de nuestra base de datos
        //idea: En función de la tarea que es pues iré a una especie de database donde tenemos los parámetros almacenados correspondientes a cada tarea.
        //Esto quizás sea demasiado complejo, lo mejor quizás sea que sea predefinido sencillito y luego ya complicarlo si eso cuando tengamos voz/demos.

        if (res.atomic_task.back()== 0){
            std::cout<<"Where should i move it? (defined_pos: close_cup, close_mouth...) "<<std::endl;
            //¿Cómo hacerlo aqui? ir introduciendo posición x, y, z... orientación x,y,z,w, o simplemente introducirlo como algo abstracto del tipo (cerca suelo etc...)
            res.pose.push_back(readkeyboard());
        }
        else{
         res.pose.push_back("NULL");
        }


        std::cout<<"Which arm should i use to perform this task? If does not matter leave it in blank"<<std::endl;
        res.move_group.push_back(readkeyboard());
        i++;
    }
    //At the end of the communication we have defined the high level structure (pick.object+task+place.object) and the low level strcuture with the movements of the different move_groups
    
    //Una vez hehco esto lo interesante sería dejar este nodo funcionando por si el usario quiere añadirle información o algún feedback.
    std::cout<<"Thank you for your help!"<<std::endl;
    return true;
}
/* TEST CODE
bool online_communication(teo_moveit::communication_srv::Request  &req, teo_moveit::communication_srv::Response &res){
    /*The porpouse of this service is to allow the robot to communicate with the user to get some feedback
    * The idea is that the robot can ask some predefined question (example 1, 2, 3...) and then the user has to answer this with yes or no
    * or maybe more complex answers later on.
    * First ideas:
    * 1) Am i doing it right? Y/N
    * 2) Could you explain me again this part in a more detailed way? Y/N if yes go back to init form with that part
    * 3)... things like that, at first we only gonna use 1) since is the most simple and is quiete effective
    * */
//}

int main(int argc, char **argv){
    ros::init(argc, argv, "communication");
    ros::NodeHandle nh;
    //communication res;
    //string object;
    //string atomic_task[20];
    //string objeto[20];
    //string entorno[20];
    //string move_group[20];
    ros::ServiceServer service = nh.advertiseService("communication", communication);
    //ros::Publisher com_pub = n.advertise<communication>("communication", 1000);

    ros::spin();

    return 0;

}
