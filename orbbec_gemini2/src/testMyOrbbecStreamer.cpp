#include "orbbec_gemini2/MyOrbbecStreamer.h"

int main(int argc, char *argv[]){
  MyOrbbecStreamer streamer(argc, argv) ;
  // call any setX() before any createXPipeline() methods

  streamer.runStream();
  return 0;
}
