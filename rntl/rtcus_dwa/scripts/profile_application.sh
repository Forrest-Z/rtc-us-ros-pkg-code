#!/bin/bash
#reference: http://comments.gmane.org/gmane.linux.oprofile/8049

export NOMBRE_APP=$1
#EVENTS="--event=CPU_CLK_UNHALTED:10000 --event=L2_RQSTS:466500 --event=L2_DATA_RQSTS:466500 --event=L2_WRITE:466500"
EVENTS="--event=CPU_CLK_UNHALTED:90000:0:1:1"
#--ctr0-event=CPU_CLK_UNHALTED --ctr0-count=600000
#EVENTS="--event=default"
roscd rtcus_gpu_dwa
if [ -e $1 ]
then 

	export NOMBRE_DIRECTORIO=`rospack find rtcus_dwa`/scripts/tests_oprofile
	export NOMBRE_SESION=`date +%d_%h_%H_%M_%S`

	cd -
	echo "--------"
	echo "rtcu_us: aplicación a analizar: $NOMBRE_APP" 
	echo "rtcu_us: guardando los resultados en $NOMBRE_DIRECTORIO" 
	echo "rtcu_us: directorio de sesion donde se guardaran los resultados es $NOMBRE_SESION"	
	echo "--------"
	sudo opcontrol --init
	sudo opcontrol --reset
	echo "rtcu_us: activando eventos de captura de contadores: $EVENTS"
	sudo opcontrol --setup $EVENTS
	sudo opcontrol --no-vmlinux  --session-dir=$NOMBRE_DIRECTORIO --image=$NOMBRE_APP --separate=library,cpu,thread --callgraph 3
	echo "rtcu_us: mostrando la configuración actual antes de comenzar el profiling..."
	sudo opcontrol --status
	echo "---------------"
	sudo opcontrol --start-daemon
	sudo opcontrol --start -V
	echo "rtcu_us: asegura que la aplicación está corriendo.. tras terminar el análisis pulse una tecla para terminar"
	read -n1 kbd
	echo "--------"
	echo "rtcu_us: parando el demonio opcontrol..."
	sudo opcontrol --stop
	echo "rtcu_us: mostrando volcado de resultados..."
	sudo opcontrol --dump
	sudo opreport -l $NOMBRE_APP
	echo "rtcu_us: guardando los datos de la sesion..."
	sudo opcontrol --save=$NOMBRE_SESION
	#sudo opannotate --source $NOMBRE_APP
	sudo opcontrol --shutdown 
	echo "rtcu_us: generando reporte por pantalla..."
	execute_command="opreport --symbols --debug-info --session-dir=$NOMBRE_DIRECTORIO session:$NOMBRE_SESION --merge=cpu,tgid"
	echo $execute_command
	opreport --symbols --debug-info --session-dir=$NOMBRE_DIRECTORIO session:$NOMBRE_SESION --merge=cpu,tgid
	xmlfile=`rospack find rtcus_gpu_dwa`/results/$NOMBRE_SESION.xml
	export PROFILE_RESULT=$xmlfile
	opreport --symbols --debug-info --demangle=smart --session-dir=$NOMBRE_DIRECTORIO session:$NOMBRE_SESION --xml -c > $xmlfile
	cp $xmlfile `rospack find rtcus_gpu_dwa`/results/latest.xml

	echo "rtcu_us: sesion almacenada en $NOMBRE_DIRECTORIO con el nombre $NOMBRE_SESION"

else
	echo "rtcu_us: el programa a ejecutar no existe: $NOMBRE_APP"

fi

