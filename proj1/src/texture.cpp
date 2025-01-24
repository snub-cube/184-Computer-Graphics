#include "texture.h"
#include "CGL/color.h"

#include <cmath>
#include <algorithm>

namespace CGL {

  //helper, linear interpolation 1D , compute color on x position
  Color lerp(float x, Color c0, Color c1) {
        Color color = c0 + x * (c1 + (-1 * c0));
        return color;
    }


  Color Texture::sample(const SampleParams& sp) {
    // TODO: Task 6: Fill this in.




    Color color;
      float level = get_level(sp);
      //level should be in range [0, mipmap.size()-1]
      level = max( 0.0f, min(level, float(mipmap.size()-1)));

      if (sp.lsm == L_ZERO) {
          level = 0;
          if (sp.psm == P_NEAREST) {
              color = sample_nearest(sp.p_uv, int(level));
          }
          else if (sp.psm == P_LINEAR) {
              color = sample_bilinear(sp.p_uv, int(level));
          }
      }

      if (sp.lsm == L_NEAREST) {
          level = round(level);
          if (sp.psm == P_NEAREST) {
              color = sample_nearest(sp.p_uv, int(level));
          }
          else if (sp.psm == P_LINEAR) {
              color = sample_bilinear(sp.p_uv, int(level));
          }
      }

      if (sp.lsm == L_LINEAR) {
          int level_up = (int)ceil(level);
          int level_down = (int)floor(level);
     
          Color color_up, color_down;
          if (sp.psm == P_NEAREST) {
              color_up = sample_nearest(sp.p_uv, level_up);
              color_down = sample_nearest(sp.p_uv, level_down);
          }
          else if (sp.psm == P_LINEAR) {
              color_up = sample_bilinear(sp.p_uv, level_up);
              color_down = sample_bilinear(sp.p_uv, level_down);
          }
          color = lerp(level - level_down, color_down, color_up);
      }

      return color;
  }

  float Texture::get_level(const SampleParams& sp) {
    // TODO: Task 6: Fill this in.

    //Calculate the difference vectors 
    Vector2D dx_uv = sp.p_dx_uv - sp.p_uv; 
    Vector2D dy_uv = sp.p_dy_uv - sp.p_uv;
    //Scale up by the width and height of full-resolution texture image
    Vector2D duv_dx (dx_uv[0] * (width-1), dx_uv[1] * (height-1));
    Vector2D duv_dy (dy_uv[0] * (width-1), dy_uv[1] * (height-1));
    //Calculate level:  D = log2(L)
    float L = max(duv_dx.norm(), duv_dy.norm());
    float D = log2(L);
    return D;
  }

  Color MipLevel::get_texel(int tx, int ty) {
    return Color(&texels[tx * 3 + ty * width * 3]);
  }

  Color Texture::sample_nearest(Vector2D uv, int level) {
    // TODO: Task 5: Fill this in.

    auto& mip = mipmap[level];
    int u = (int)round(uv[0] * (mip.width-1));
    int v = (int)round(uv[1] * (mip.height-1));

    Color color = mip.get_texel(u, v);
    return color;
  }

  Color Texture::sample_bilinear(Vector2D uv, int level) {
    // TODO: Task 5: Fill this in.
    auto& mip = mipmap[level];
    float u = uv[0] * (mip.width-1);
    float v = uv[1] * (mip.height-1);

    int u0 = (int)floor(u);
    int v0 = (int)floor(v);
    int u1 = (int)ceil(u);
    int v1 = (int)ceil(v);

    Color c00 = mip.get_texel(u0, v0);
    Color c01 = mip.get_texel(u0, v1);
    Color c10 = mip.get_texel(u1, v0);
    Color c11 = mip.get_texel(u1, v1);

    Color c0 = lerp(u - u0, c00, c10); //horizontal direction, s
    Color c1 = lerp(u - u0, c01, c11);
    Color color = lerp(v - v0, c0, c1);//vertical direction, l
    return color;
  }



  /****************************************************************************/

  // Helpers

  inline void uint8_to_float(float dst[3], unsigned char* src) {
    uint8_t* src_uint8 = (uint8_t*)src;
    dst[0] = src_uint8[0] / 255.f;
    dst[1] = src_uint8[1] / 255.f;
    dst[2] = src_uint8[2] / 255.f;
  }

  inline void float_to_uint8(unsigned char* dst, float src[3]) {
    uint8_t* dst_uint8 = (uint8_t*)dst;
    dst_uint8[0] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[0])));
    dst_uint8[1] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[1])));
    dst_uint8[2] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[2])));
  }

  void Texture::generate_mips(int startLevel) {

    // make sure there's a valid texture
    if (startLevel >= mipmap.size()) {
      std::cerr << "Invalid start level";
    }

    // allocate sublevels
    int baseWidth = mipmap[startLevel].width;
    int baseHeight = mipmap[startLevel].height;
    int numSubLevels = (int)(log2f((float)max(baseWidth, baseHeight)));

    numSubLevels = min(numSubLevels, kMaxMipLevels - startLevel - 1);
    mipmap.resize(startLevel + numSubLevels + 1);

    int width = baseWidth;
    int height = baseHeight;
    for (int i = 1; i <= numSubLevels; i++) {

      MipLevel& level = mipmap[startLevel + i];

      // handle odd size texture by rounding down
      width = max(1, width / 2);
      //assert (width > 0);
      height = max(1, height / 2);
      //assert (height > 0);

      level.width = width;
      level.height = height;
      level.texels = vector<unsigned char>(3 * width * height);
    }

    // create mips
    int subLevels = numSubLevels - (startLevel + 1);
    for (int mipLevel = startLevel + 1; mipLevel < startLevel + subLevels + 1;
      mipLevel++) {

      MipLevel& prevLevel = mipmap[mipLevel - 1];
      MipLevel& currLevel = mipmap[mipLevel];

      int prevLevelPitch = prevLevel.width * 3; // 32 bit RGB
      int currLevelPitch = currLevel.width * 3; // 32 bit RGB

      unsigned char* prevLevelMem;
      unsigned char* currLevelMem;

      currLevelMem = (unsigned char*)&currLevel.texels[0];
      prevLevelMem = (unsigned char*)&prevLevel.texels[0];

      float wDecimal, wNorm, wWeight[3];
      int wSupport;
      float hDecimal, hNorm, hWeight[3];
      int hSupport;

      float result[3];
      float input[3];

      // conditional differentiates no rounding case from round down case
      if (prevLevel.width & 1) {
        wSupport = 3;
        wDecimal = 1.0f / (float)currLevel.width;
      }
      else {
        wSupport = 2;
        wDecimal = 0.0f;
      }

      // conditional differentiates no rounding case from round down case
      if (prevLevel.height & 1) {
        hSupport = 3;
        hDecimal = 1.0f / (float)currLevel.height;
      }
      else {
        hSupport = 2;
        hDecimal = 0.0f;
      }

      wNorm = 1.0f / (2.0f + wDecimal);
      hNorm = 1.0f / (2.0f + hDecimal);

      // case 1: reduction only in horizontal size (vertical size is 1)
      if (currLevel.height == prevLevel.height) {
        //assert (currLevel.height == 1);

        for (int i = 0; i < currLevel.width; i++) {
          wWeight[0] = wNorm * (1.0f - wDecimal * i);
          wWeight[1] = wNorm * 1.0f;
          wWeight[2] = wNorm * wDecimal * (i + 1);

          result[0] = result[1] = result[2] = 0.0f;

          for (int ii = 0; ii < wSupport; ii++) {
            uint8_to_float(input, prevLevelMem + 3 * (2 * i + ii));
            result[0] += wWeight[ii] * input[0];
            result[1] += wWeight[ii] * input[1];
            result[2] += wWeight[ii] * input[2];
          }

          // convert back to format of the texture
          float_to_uint8(currLevelMem + (3 * i), result);
        }

        // case 2: reduction only in vertical size (horizontal size is 1)
      }
      else if (currLevel.width == prevLevel.width) {
        //assert (currLevel.width == 1);

        for (int j = 0; j < currLevel.height; j++) {
          hWeight[0] = hNorm * (1.0f - hDecimal * j);
          hWeight[1] = hNorm;
          hWeight[2] = hNorm * hDecimal * (j + 1);

          result[0] = result[1] = result[2] = 0.0f;
          for (int jj = 0; jj < hSupport; jj++) {
            uint8_to_float(input, prevLevelMem + prevLevelPitch * (2 * j + jj));
            result[0] += hWeight[jj] * input[0];
            result[1] += hWeight[jj] * input[1];
            result[2] += hWeight[jj] * input[2];
          }

          // convert back to format of the texture
          float_to_uint8(currLevelMem + (currLevelPitch * j), result);
        }

        // case 3: reduction in both horizontal and vertical size
      }
      else {

        for (int j = 0; j < currLevel.height; j++) {
          hWeight[0] = hNorm * (1.0f - hDecimal * j);
          hWeight[1] = hNorm;
          hWeight[2] = hNorm * hDecimal * (j + 1);

          for (int i = 0; i < currLevel.width; i++) {
            wWeight[0] = wNorm * (1.0f - wDecimal * i);
            wWeight[1] = wNorm * 1.0f;
            wWeight[2] = wNorm * wDecimal * (i + 1);

            result[0] = result[1] = result[2] = 0.0f;

            // convolve source image with a trapezoidal filter.
            // in the case of no rounding this is just a box filter of width 2.
            // in the general case, the support region is 3x3.
            for (int jj = 0; jj < hSupport; jj++)
              for (int ii = 0; ii < wSupport; ii++) {
                float weight = hWeight[jj] * wWeight[ii];
                uint8_to_float(input, prevLevelMem +
                  prevLevelPitch * (2 * j + jj) +
                  3 * (2 * i + ii));
                result[0] += weight * input[0];
                result[1] += weight * input[1];
                result[2] += weight * input[2];
              }

            // convert back to format of the texture
            float_to_uint8(currLevelMem + currLevelPitch * j + 3 * i, result);
          }
        }
      }
    }
  }

}
