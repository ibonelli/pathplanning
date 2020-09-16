#PYRUN="main.py"
PYRUN="main_astar.py"

FILE="world01"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world02"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world03"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world04"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world11"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world12"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world13"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world14"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world21"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world22"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world23"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world24"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world31"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world32"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

FILE="world33"
echo "Running "${FILE}" ================"
python3 ${PYRUN} ${FILE}.csv
mv navigation.log ${FILE}_navigation.log

cat *.txt > All_Runs.txt
