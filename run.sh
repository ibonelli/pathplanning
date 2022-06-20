PYRUN="main.py"
#PYRUN="main_astar.py"
DIR="logs_and_support_files/latest_run"

function run_world {
	echo "Running "${FILE}" ================"
	SAVE=${DIR}"/"${FILE}"/"
	python3 ${PYRUN} ${FILE}.csv
	mv navigation.log ${FILE}_navigation.log
	mkdir -p ${SAVE}
	mv ${FILE}_* ${SAVE}
}

while read WFNAME; do
	FILE="$WFNAME"
	run_world
done <worlds_to_run.txt

while read WFNAME; do
	FILE="$WFNAME"
	run_world
done <worlds_to_run2.txt

cat *.txt > All_Runs.txt
