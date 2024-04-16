#include <mujoco/mujoco.h>
#include <cstdio>


int main(int argc, char **argv) {
    //mjModel *m = mj_loadXML("box.xml", NULL, 0);
    //mjData *d = mj_makeData(m);

    //// Simulate for 1000 time steps
    //for (int i = 0; i < 1000; i++) {
        //mj_step(m, d);
    //}

    // Print "Hello World" to the console
    printf("Hello World\n");

     //Free the memory used by the model and data
    //mj_deleteModel(m);
    //mj_deleteData(d);

    return 0;
}


