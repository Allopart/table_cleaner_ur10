#ifndef PARSER_H
#define PARSER_H
#include "network.h"

#if defined (__cplusplus)
extern "C" {
#endif

network * parse_network_cfg(char *filename);
void save_network(network * net, char *filename);
void save_weights(network * net, char *filename);
void save_weights_upto(network * net, char *filename, int cutoff);
void save_weights_double(network * net, char *filename);
void load_weights(network *net, char *filename);
void load_weights_upto(network *net, char *filename, int cutoff);

#if defined (__cplusplus)
}
#endif

#endif
