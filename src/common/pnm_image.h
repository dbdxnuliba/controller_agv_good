#ifndef __PNM_IMAGE_H__
#define __PNM_IMAGE_H__

#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>

enum class pnm_t 
{ 
    P1=1, 
    P2, 
    P3, 
    P4, 
    P5, 
    P6 
};

uint8_t *read_pnm_image(int *width, int *height, std::string filename);
void write_pnm_image(const uint8_t *img, int width, int height, std::string filename, pnm_t format=pnm_t::P2);


#endif /* __PNM_IMAGE_H__ */
