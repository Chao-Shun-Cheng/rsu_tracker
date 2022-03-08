#include <rsu_tracker/imm_ukf_pda.h>

int main(int argc, char **argv)
{
    srand(time(NULL));
    ros::init(argc, argv, "rsu_tracker");
    ImmUkfPda app;
    app.run();
    ros::spin();
    return 0;
}