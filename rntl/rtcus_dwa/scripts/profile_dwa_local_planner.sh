roscd rtcus_dwa
source ./scripts/profile_application.sh `rospack find rtcus_dwa`/bin/dwa_local_planner
roscd rtcus_dwa
cd ./src/ResultQueries

echo "======result======"
mono bin/Debug/ResultQueries.exe $PROFILE_RESULT

