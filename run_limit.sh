PYRUN="main.py"
#PYRUN="main_astar.py"
DIR="logs_and_support_files/Limits"

function run_limit {
	echo "Running "${FILE}" ================"
	SAVE=${DIR}"/"${FILE}"/vision_limit_"${LIMIT}"/"
	python3 ${PYRUN} -vl ${LIMIT} -rn vision_limit ${FILE}.csv
	mv navigation.log ${FILE}_navigation.log
	mkdir -p ${SAVE}
	mv ${FILE}_* ${SAVE}
}

function run_lidar {
	echo "Running "${FILE}" ================"
	SAVE=${DIR}"/"${FILE}"/lidar_steps_"${LIMIT}"/"
	python3 ${PYRUN} -ls ${LIMIT} -rn lidar_steps ${FILE}.csv
	mv navigation.log ${FILE}_navigation.log
	mkdir -p ${SAVE}
	mv ${FILE}_* ${SAVE}
}

function run_world_limit {
	LIMIT="05"
	run_limit
	LIMIT="10"
	run_limit
	LIMIT="20"
	run_limit
	#LIMIT="30"
	#run_limit
}

function run_world_lidar {
	LIMIT="08"
	run_lidar
	LIMIT="16"
	run_lidar
	LIMIT="32"
	run_lidar
	#LIMIT="64"
	#run_lidar
	#LIMIT="128"
	#run_lidar
}

FILE="world11"
run_world_limit
run_world_lidar

cd ${DIR}"/"${FILE}
find ./ -iname "*report*" | xargs tail -n +1 > ${FILE}_report_full.txt
find ./ -iname "*nav.png" | xargs -I{} cp -u {} ./
find ./ -iname "*objs.png" | xargs -I{} cp -u {} ./
