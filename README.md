# README #
                                   
Manual de Usuario
=================

1. Instalación de las principales bibliotecas y paquetes necesarios:<br/>
        a)  OpenCV 2.4: Cómo instalar OpenCV (https://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html)<br/>
        b)  OpenGL 2.0: Cómo instalar OpenGL (https://en.wikibooks.org/wiki/OpenGL_Programming/Installation/Linux)<br/>
        c)  OSVR: Cómo instalar OSVR (https://osvr.github.io/doc/installing/linux/)<br/>
        d)  ROS Indigo: Cómo instalar ROS (http://wiki.ros.org/indigo/Installation/Ubuntu)<br/><br/>
2.  Creacion de un workspacecatkin, dentro del entorno de ROS (más detalles decomo realizar este paso en la documentacion oficial de ROS(http://wiki.ros.org/catkin/Tutorials/create_a_workspace)):<br/>
        a)  Crear carpeta “catkinws”: mkdir catkinws<br/>
        b)  Acceder a la carpeta: cd catkinws<br/>
        c)  Crear carpeta “src”: mkdir src<br/>
        d)  Crear paquete del proyecto: catkin_create_pkg nombre_proyecto<br/>
        e)  Acceder a la carpeta del proyecto:cd src/nombre_proyecto<br/><br/>
3.  Clonar proyecto disponible en github: https://github.com/christianjaka94/osvr_unizar.git<br/><br/>
4.  Compilación de los fuentes:<br/>
        a)  Desde el directorio raız (/catkinws): catkin_make<br/><br/>
5.  Preparacion del entorno:<br/>
        a)  Encender la camara GoPro<br/>
        b)  Conectar el PC a la red WiFi de la cámara<br/>
        c)  Iniciar grabacion<br/><br/>
6.  Ejecución del programa:<br/>
        a)  Lanzar pan-tilt en nueva terminal:<br/><br/>
                  i)Ir al directorio base: cd $HOME/catkin_ws/devel/lib<br/>
                  ii)Ejecutar pan-tilt (se calibra sólo): roslaunch asr_flir_ptu_driver ptu_left.launch<br/><br/>
        b)  Lanzar servidor OSVR en nueva terminal:<br/><br/>
                  i)Ir al directorio base:cd $HOME/OSVR/OSVR-Core/apps/sample-configs<br/>
                  ii)Lanzar servidor: sudo osvr_server osvr_server_config.renderManager.HDKv2.0.extended.json<br/><br/>
        c)  Ejecucion procesos principales:<br/><br/>
                  i)Ir al directorio donde se encuentran los ejecutables: cd devel/lib/osvr/<br/>
                  ii)Lanzar proceso-pantilt:./Pantilt<br/>
                  iii)Lanzar proceso-gafas:./Tfg<br/>

