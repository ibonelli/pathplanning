PYRUN="main.py"
#PYRUN="main_astar.py"
DIR="nopath_run"

function run_world {
	echo "Running "${FILE}" ================"
	SAVE=${DIR}"/"${FILE}"/VL"${VL}"LS"${LS}"/"
	python3 ${PYRUN} -vl ${VL} -ls ${LS} ${FILE}.csv
	mv navigation.log ${FILE}_navigation.log
	mkdir -p ${SAVE}
	mv ${FILE}_* ${SAVE}
}

VL="10"
LS="08"
FILE="world96.csv"
run_world

VL="30"
LS="32"
FILE="world96.csv"

VL="10"
LS="08"
FILE="world99.csv"
run_world

VL="30"
LS="32"
FILE="world99.csv"
