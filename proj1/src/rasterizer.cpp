#include "rasterizer.h"

using namespace std;

namespace CGL {

  RasterizerImp::RasterizerImp(PixelSampleMethod psm, LevelSampleMethod lsm,
    size_t width, size_t height,
    unsigned int sample_rate) {
    this->psm = psm;
    this->lsm = lsm;
    this->width = width;
    this->height = height;
    this->sample_rate = sample_rate;

    sample_buffer.resize(width * height * sample_rate, Color::White);
  }

  // Used by rasterize_point and rasterize_line
  void RasterizerImp::fill_pixel(size_t x, size_t y, Color c) {
    // TODO: Task 2: You might need to this function to fix points and lines (such as the black rectangle border in test4.svg)
    // NOTE: You are not required to implement proper supersampling for points and lines
    // It is sufficient to use the same color for all supersamples of a pixel for points and lines (not triangles)
    int sub_len = sqrt(sample_rate);
    for (int sub_x = 0; sub_x < sub_len; sub_x++) {
        for (int sub_y = 0; sub_y < sub_len; sub_y++) {
            int row = y * sub_len + sub_y;
            int col = x * sub_len + sub_x;
            sample_buffer[row * width * sub_len + col] = c;
        }
    }
  }

  // Rasterize a point: simple example to help you start familiarizing
  // yourself with the starter code.
  //
  void RasterizerImp::rasterize_point(float x, float y, Color color) {
    // fill in the nearest pixel
    int sx = (int)floor(x);
    int sy = (int)floor(y);

    // check bounds
    if (sx < 0 || sx >= width) return;
    if (sy < 0 || sy >= height) return;

    fill_pixel(sx, sy, color);
    return;
  }

  // Rasterize a line.
  void RasterizerImp::rasterize_line(float x0, float y0,
    float x1, float y1,
    Color color) {
    if (x0 > x1) {
      swap(x0, x1); swap(y0, y1);
    }

    float pt[] = { x0,y0 };
    float m = (y1 - y0) / (x1 - x0);
    float dpt[] = { 1,m };
    int steep = abs(m) > 1;
    if (steep) {
      dpt[0] = x1 == x0 ? 0 : 1 / abs(m);
      dpt[1] = x1 == x0 ? (y1 - y0) / abs(y1 - y0) : m / abs(m);
    }

    while (floor(pt[0]) <= floor(x1) && abs(pt[1] - y0) <= abs(y1 - y0)) {
      rasterize_point(pt[0], pt[1], color);
      pt[0] += dpt[0]; pt[1] += dpt[1];
    }
  }


  //helper
  float line_equation(float & x0, float&  y0, float&  x1, float & y1, float & x, float & y ) {
      return -(x - x0) * (y1 - y0) + (y - y0) * (x1 - x0);
  }
  
  //return true if a point(x,y) lies inside triangle with vertices (x0,y0),(x1,y1),(x2,y2) 
  bool inside_triangle(float& x0, float& y0, float& x1, float& y1, float& x2, float& y2, float& x, float& y) {
      float l1 = line_equation(x0, y0, x1, y1, x, y);
      float l2 = line_equation(x1, y1, x2, y2, x, y);
      float l3 = line_equation(x2, y2, x0, y0, x, y);
      return (l1 >= 0 && l2 >= 0 && l3 >= 0) || (l1 <= 0 && l2 <= 0 && l3 <= 0);
  }

  // Rasterize a triangle.
  void RasterizerImp::rasterize_triangle(float x0, float y0,
    float x1, float y1,
    float x2, float y2,
    Color color) {
    // TODO: Task 1: Implement basic triangle rasterization here, no supersampling
    /*
    int min_x = (int)floor(min(min(x0, x1), x2));
      int min_y = (int)floor(min(min(y0, y1), y2));
      int max_x = (int)ceil(max(max(x0, x1), x2));  
      int max_y = (int)ceil(max(max(y0, y1), y2));
      for (int x = min_x; x < max_x; x++){
          for (int y = min_y; y < max_y; y++) {
              float x_s = x + 0.5;
              float y_s = y + 0.5;
              if (inside_triangle(x0, y0, x1, y1, x2, y2, x_s, y_s)){
                  sample_buffer[y * width + x] = color;
              }
          }
      }
    */
    // TODO: Task 2: Update to implement super-sampled rasterization
    float minx = min(min(x0, x1), x2);
    float miny = min(min(y0, y1), y2);
    float maxx = max(max(x0, x1), x2);
    float maxy = max(max(y0, y1), y2);
    //keep in frame
    int min_x = max((int)floor(minx), 0);
    int min_y = max((int)floor(miny), 0);
    int max_x = min((int)ceil(maxx), (int)width);
    int max_y = min((int)ceil(maxy), (int)height);

    int sub_len = sqrt(sample_rate);           //subsamples in row or col for a pixel
    float sub_step = 1.0f / sqrt(sample_rate); //step between subsamples in row or col direction

    for (int x = min_x; x < max_x; x++) {
        for (int y = min_y; y < max_y; y++) {
            for (int sub_x = 0; sub_x < sub_len; sub_x++) {
                for (int sub_y = 0; sub_y < sub_len; sub_y++) {
                    float x_s = x + 0.5 * sub_step + sub_x * sub_step;
                    float y_s = y + 0.5 * sub_step + sub_y * sub_step;
                    if (inside_triangle(x0, y0, x1, y1, x2, y2, x_s, y_s)) {
                        int sx = x * sub_len + sub_x;
                        int sy = y * sub_len + sub_y;
                        sample_buffer[sy * width * sub_len + sx] = color;
                    }
                }
            }
        }
    }
  }

  //helper
  //return barycentric coordinates for (x,y)
  Vector3D bary_coord(float& x0, float& y0, float& x1, float& y1, float& x2, float& y2, float& x, float& y) {
      //select double type.
      double alpha = line_equation(x1, y1, x2, y2, x, y) / line_equation(x1, y1, x2, y2, x0, y0);
      double beta = line_equation(x2, y2, x0, y0, x, y) / line_equation(x2, y2, x0, y0, x1, y1);
      double gamma = 1 - alpha - beta;
      return Vector3D(alpha, beta, gamma);
  }

  void RasterizerImp::rasterize_interpolated_color_triangle(float x0, float y0, Color c0,
    float x1, float y1, Color c1,
    float x2, float y2, Color c2)
  {
    // TODO: Task 4: Rasterize the triangle, calculating barycentric coordinates and using them to interpolate vertex colors across the triangle
    // Hint: You can reuse code from rasterize_triangle
    // TODO: Task 4: Rasterize the triangle, calculating barycentric coordinates and using them to interpolate vertex colors across the triangle
    // Hint: You can reuse code from rasterize_triangle
    float minx = min(min(x0, x1), x2);
    float miny = min(min(y0, y1), y2);
    float maxx = max(max(x0, x1), x2);
    float maxy = max(max(y0, y1), y2);
    //keep in frame
    int min_x = max( (int)floor(minx), 0);
    int min_y = max( (int)floor(miny), 0);
    int max_x = min( (int)ceil(maxx), (int)width);
    int max_y = min( (int)ceil(maxy), (int)height);

    int sub_len = sqrt(sample_rate);
    float sub_step = 1.0f / sqrt(sample_rate);

    for (int x = min_x; x < max_x; x++) {
        for (int y = min_y; y < max_y; y++) {
            for (int sub_x = 0; sub_x < sub_len; sub_x++) {
                for (int sub_y = 0; sub_y < sub_len; sub_y++) {
                    float x_s = x + 0.5 * sub_step + sub_x * sub_step;
                    float y_s = y + 0.5 * sub_step + sub_y * sub_step;
                    if (inside_triangle(x0, y0, x1, y1, x2, y2, x_s, y_s)) {
                        Vector3D v_abg = bary_coord(x0, y0, x1, y1, x2, y2, x_s, y_s);
                        Color color = c0 * v_abg[0] + c1 * v_abg[1] + c2 * v_abg[2];//compute color, c0 * alpha + c1 * beta + c2 * gamma;

                        int sx = x * sub_len + sub_x;
                        int sy = y * sub_len + sub_y;
                        sample_buffer[sy * width * sub_len + sx] = color;
                    }
                }
            }
        }
    }

  }

//helper
//return uv barycentric coordinates
  Vector2D uv_coord(float& x0, float& y0, float& u0, float& v0,
      float& x1, float& y1, float& u1, float& v1,
      float& x2, float& y2, float& u2, float& v2,
      float x, float y)
  {   
      Vector3D v_abg = bary_coord(x0, y0, x1, y1, x2, y2, x, y); //get alpha,beta,gamma

      Vector2D uv; //compute uv barycentric coordinates
      uv[0] = u0 * v_abg[0] + u1 * v_abg[1] + u2 * v_abg[2];
      uv[1] = v0 * v_abg[0] + v1 * v_abg[1] + v2 * v_abg[2];
      return uv;
  }

  void RasterizerImp::rasterize_textured_triangle(float x0, float y0, float u0, float v0,
    float x1, float y1, float u1, float v1,
    float x2, float y2, float u2, float v2,
    Texture& tex)
  {
    // TODO: Task 5: Fill in the SampleParams struct and pass it to the tex.sample function.
    // TODO: Task 6: Set the correct barycentric differentials in the SampleParams struct.
    // Hint: You can reuse code from rasterize_triangle/rasterize_interpolated_color_triangle


    float minx = min(min(x0, x1), x2);
    float miny = min(min(y0, y1), y2);
    float maxx = max(max(x0, x1), x2);
    float maxy = max(max(y0, y1), y2);
    //keep in frame
    int min_x = max((int)floor(minx), 0);
    int min_y = max((int)floor(miny), 0);
    int max_x = min((int)ceil(maxx), (int)width);
    int max_y = min((int)ceil(maxy), (int)height);

    int sub_len = sqrt(sample_rate);
    float sub_step = 1.0f /sqrt(sample_rate);

    SampleParams sp;
    sp.lsm = lsm;
    sp.psm = psm;

    for (int x = min_x; x < max_x; x++) {
        for (int y = min_y; y < max_y; y++) {
            for (int sub_x = 0; sub_x < sub_len; sub_x++) {
                for (int sub_y = 0; sub_y < sub_len; sub_y++) {
                    float x_s = x + 0.5 * sub_step + sub_x * sub_step;
                    float y_s = y + 0.5 * sub_step + sub_y * sub_step;
                    if (inside_triangle(x0, y0, x1, y1, x2, y2, x_s, y_s) && (x_s + 1 < width) && (y_s + 1 < height)) {
                        //update sp
                        sp.p_uv = uv_coord(x0, y0, u0, v0, x1, y1, u1, v1, x2, y2, u2, v2, x_s, y_s);
                        sp.p_dx_uv = uv_coord(x0, y0, u0, v0, x1, y1, u1, v1, x2, y2, u2, v2, x_s + 1, y_s);
                        sp.p_dy_uv = uv_coord(x0, y0, u0, v0, x1, y1, u1, v1, x2, y2, u2, v2, x_s, y_s + 1);
                        Color color = tex.sample(sp);

                        int sx = x * sub_len + sub_x;
                        int sy = y * sub_len + sub_y;
                        sample_buffer[sy * width * sub_len + sx] = color;
                    }
                }
            }
        }
    }
  }

  void RasterizerImp::set_sample_rate(unsigned int rate) {
    // TODO: Task 2: You may want to update this function for supersampling support

    this->sample_rate = rate;


    // this->sample_buffer.resize(width * height, Color::White);
    this->sample_buffer.resize(width * height * rate, Color::White);
  }


  void RasterizerImp::set_framebuffer_target(unsigned char* rgb_framebuffer,
    size_t width, size_t height)
  {
    // TODO: Task 2: You may want to update this function for supersampling support

    this->width = width;
    this->height = height;
    this->rgb_framebuffer_target = rgb_framebuffer;


    this->sample_buffer.resize(width * height * sample_rate, Color::White);
  }


  void RasterizerImp::clear_buffers() {
    std::fill(rgb_framebuffer_target, rgb_framebuffer_target + 3 * width * height, 255);
    std::fill(sample_buffer.begin(), sample_buffer.end(), Color::White);
  }


  // This function is called at the end of rasterizing all elements of the
  // SVG file.  If you use a supersample buffer to rasterize SVG elements
  // for antialising, you could use this call to fill the target framebuffer
  // pixels from the supersample buffer data.
  //
  void RasterizerImp::resolve_to_framebuffer() {
    // TODO: Task 2: You will likely want to update this function for supersampling support
    int sub_len = sqrt(sample_rate);

    for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {
        //Color col = sample_buffer[y * width + x];
   
        // sum and average color for all subsamples in a pixel
        Color col = Color::Black;  
        for (int c = x * sub_len; c < x * sub_len + sub_len; c++) {
            for (int r = y* sub_len; r < y* sub_len + sub_len; r++) {
                col = col + sample_buffer[r * width* sub_len + c];
            }
        }
        col = col * (1.0f / sample_rate);  //Color * float scale. average color

        for (int k = 0; k < 3; ++k) {
            this->rgb_framebuffer_target[3 * (y * width + x) + k] = (&col.r)[k] * 255;

        }
      }
    }

  }

  Rasterizer::~Rasterizer() { }


}// CGL
