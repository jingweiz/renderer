#include "renderer/renderer.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "renderer_node");
    ros::AsyncSpinner spinner(4);

  	renderer::Renderer *renderer = new renderer::Renderer();
  	renderer->initROS();

    spinner.start();
    ros::waitForShutdown();
    spinner.stop();

    delete renderer;
    renderer = NULL;
}
