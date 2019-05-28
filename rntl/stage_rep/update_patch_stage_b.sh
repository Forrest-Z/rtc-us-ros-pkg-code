make clean
rm -R lib64
rm -R stage
rm -R rtcus_stage.patch
wget http://pr.willowgarage.com/downloads/stage-cffb7bf9819469e38396c7711cfde3f4c9573a0e.tar.gz .;
tar -zxvf ./stage-cffb7bf9819469e38396c7711cfde3f4c9573a0e.tar.gz stage;
tar -zxvf ./stage-cffb7bf9819469e38396c7711cfde3f4c9573a0e.tar.gz stage_b;
cp ./src/stage_patch/stage.hh stage_b/libstage/stage.hh
cp ./src/stage_patch/model.cc stage_b/libstage/model.cc
cp ./src/stage_patch/model_position.cc stage_b/libstage/model_position.cc

cd stage
diff -pur . ../stage_b > ../rtcus_stage.patch
cd ..
rm -R stage
rm stage-cffb7bf9819469e38396c7711cfde3f4c9573a0e.tar.gz
make
