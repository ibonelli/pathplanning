PYRUN="main.py"
#PYRUN="main_astar.py"
DIR="logs_and_support_files/Limits"

function run_limit {
	echo "Running "${FILE}" ================"
	SAVE=${DIR}"/"${FILE}"/vision_limit_"${LIMIT}"/"
	python3 ${PYRUN} ${FILE}.csv ${LIMIT}
	mv navigation.log ${FILE}_navigation.log
	mkdir -p ${SAVE}
	mv ${FILE}_* ${SAVE}
}

function run_world {
	LIMIT="05"
	run_limit
	LIMIT="10"
	run_limit
	LIMIT="20"
	run_limit
	LIMIT="30"
	run_limit
}

FILE="world22"
run_world
