#pragma once

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <string>

#include "liblvgl/misc/lv_color.h"
#include "liblvgl/widgets/canvas/lv_canvas.h"

#include "pros/rtos.h"

/**
 * MIT License
 * Copyright (c) 2025 halp1
 * Original: https://github.com/theol0403/gif-pros
 */

namespace lexlib::helpers::gif {
  namespace core {
		#define MIN(A, B) ((A) < (B) ? (A) : (B))
		#define MAX(A, B) ((A) > (B) ? (A) : (B))
    constexpr int BYTES_PER_PIXEL = 4;

    struct Palette {
      int size;
      uint8_t colors[0x100 * 3];
    };

    struct GCE {
      uint16_t delay;
      uint8_t tindex;
      uint8_t disposal;
      int input;
      int transparency;
    };

    struct GIF {
      FILE *fp;
      off_t anim_start;
      uint16_t width, height;
      uint16_t depth;
      uint16_t loop_count;
      GCE gce;
      Palette *palette;
      Palette lct, gct;
      void (*plain_text)(GIF *gif, uint16_t tx, uint16_t ty, uint16_t tw, uint16_t th, uint8_t cw, uint8_t ch, uint8_t fg, uint8_t bg);
      void (*comment)(GIF *gif);
      void (*application)(GIF *gif, char id[8], char auth[3]);
      uint16_t fx, fy, fw, fh;
      uint8_t bgindex;
      uint8_t *canvas, *frame;
    };

    struct Entry {
      uint16_t length;
      uint16_t prefix;
      uint8_t suffix;
    };

    struct Table {
      int bulk;
      int nentries;
      Entry *entries;
    };

    inline uint16_t read_num(FILE *fp) {
      uint8_t bytes[2];

      fread(bytes, 1, 2, fp);
      return bytes[0] + (((uint16_t)bytes[1]) << 8);
    }

    inline GIF *open_gif(FILE *fp) {
      uint8_t sigver[3];
      uint16_t width, height, depth;
      uint8_t fdsz, bgidx, aspect;
      int gct_sz;
      GIF *gif = nullptr;

      if (fp == nullptr)
        return nullptr;

      fread(sigver, 1, 3, fp);
      if (memcmp(sigver, "GIF", 3) != 0) {
        fprintf(stderr, "invalid signature\n");
        goto fail;
      }

      fread(sigver, 1, 3, fp);
      if (memcmp(sigver, "89a", 3) != 0) {
        fprintf(stderr, "invalid version\n");
        goto fail;
      }

      width = read_num(fp);
      height = read_num(fp);

      fread(&fdsz, 1, 1, fp);

      if (!(fdsz & 0x80)) {
        fprintf(stderr, "no global color table\n");
        goto fail;
      }

      depth = ((fdsz >> 4) & 7) + 1;

      gct_sz = 1 << ((fdsz & 0x07) + 1);

      fread(&bgidx, 1, 1, fp);

      fread(&aspect, 1, 1, fp);

      gif = static_cast<GIF *>(
          calloc(1, sizeof(*gif) + (BYTES_PER_PIXEL + 1) * width * height)
      );
      if (!gif)
        goto fail;
      gif->fp = fp;
      gif->width = width;
      gif->height = height;
      gif->depth = depth;

      gif->gct.size = gct_sz;
      fread(gif->gct.colors, 1, 3 * gif->gct.size, fp);
      gif->palette = &gif->gct;
      gif->bgindex = bgidx;
      gif->canvas = (uint8_t *)&gif[1];
      gif->frame = &gif->canvas[BYTES_PER_PIXEL * width * height];
      if (gif->bgindex)
        memset(gif->frame, gif->bgindex, gif->width * gif->height);
      gif->anim_start = ftell(fp);
      goto ok;
    fail:
      fclose(fp);
    ok:
      return gif;
    }

    inline void discard_sub_blocks(GIF *gif) {
      uint8_t size;

      do {
        fread(&size, 1, 1, gif->fp);
        fseek(gif->fp, size, SEEK_CUR);
      } while (size);
    }

    inline void read_plain_text_ext(GIF *gif) {
      if (gif->plain_text) {
        uint16_t tx, ty, tw, th;
        uint8_t cw, ch, fg, bg;
        off_t sub_block;
        fseek(gif->fp, 1, SEEK_CUR);
        tx = read_num(gif->fp);
        ty = read_num(gif->fp);
        tw = read_num(gif->fp);
        th = read_num(gif->fp);
        fread(&cw, 1, 1, gif->fp);
        fread(&ch, 1, 1, gif->fp);
        fread(&fg, 1, 1, gif->fp);
        fread(&bg, 1, 1, gif->fp);
        sub_block = ftell(gif->fp);
        gif->plain_text(gif, tx, ty, tw, th, cw, ch, fg, bg);
        fseek(gif->fp, sub_block, SEEK_SET);
      } else {
        fseek(gif->fp, 13, SEEK_CUR);
      }

      discard_sub_blocks(gif);
    }

    inline void read_graphic_control_ext(GIF *gif) {
      uint8_t rdit;

      fseek(gif->fp, 1, SEEK_CUR);
      fread(&rdit, 1, 1, gif->fp);
      gif->gce.disposal = (rdit >> 2) & 3;
      gif->gce.input = rdit & 2;
      gif->gce.transparency = rdit & 1;
      gif->gce.delay = read_num(gif->fp);
      fread(&gif->gce.tindex, 1, 1, gif->fp);

      fseek(gif->fp, 1, SEEK_CUR);
    }

    inline void read_comment_ext(GIF *gif) {
      if (gif->comment) {
        off_t sub_block = ftell(gif->fp);
        gif->comment(gif);
        fseek(gif->fp, sub_block, SEEK_SET);
      }

      discard_sub_blocks(gif);
    }

    inline void read_application_ext(GIF *gif) {
      char app_id[8];
      char app_auth_code[3];

      fseek(gif->fp, 1, SEEK_CUR);

      fread(app_id, 1, 8, gif->fp);

      fread(app_auth_code, 1, 3, gif->fp);
      if (!strncmp(app_id, "NETSCAPE", sizeof(app_id))) {
        fseek(gif->fp, 2, SEEK_CUR);
        gif->loop_count = read_num(gif->fp);

        fseek(gif->fp, 1, SEEK_CUR);
      } else if (gif->application) {
        off_t sub_block = ftell(gif->fp);
        gif->application(gif, app_id, app_auth_code);
        fseek(gif->fp, sub_block, SEEK_SET);
        discard_sub_blocks(gif);
      } else {
        discard_sub_blocks(gif);
      }
    }

    inline void read_ext(GIF *gif) {
      uint8_t label;

      fread(&label, 1, 1, gif->fp);
      switch (label) {
        case 0x01:
          read_plain_text_ext(gif);
          break;
        case 0xF9:
          read_graphic_control_ext(gif);
          break;
        case 0xFE:
          read_comment_ext(gif);
          break;
        case 0xFF:
          read_application_ext(gif);
          break;
        default:
          fprintf(stderr, "unknown extension: %02X\n", label);
      }
    }

    inline Table *new_table(int key_size) {
      int key;
      int init_bulk = MAX(1 << (key_size + 1), 0x100);
      Table *table =
          static_cast<Table *>(malloc(sizeof(*table) + sizeof(Entry) * init_bulk));
      if (table) {
        table->bulk = init_bulk;
        table->nentries = (1 << key_size) + 2;
        table->entries = (Entry *)&table[1];
        for (key = 0; key < (1 << key_size); key++)
          table->entries[key] = (Entry){1, 0xFFF, static_cast<uint8_t>(key)};
      }
      return table;
    }

    inline int add_entry(Table **tablep, uint16_t length, uint16_t prefix, uint8_t suffix) {
      Table *table = *tablep;
      if (table->nentries == table->bulk) {
        table->bulk *= 2;
        table = static_cast<Table *>(
            realloc(table, sizeof(*table) + sizeof(Entry) * table->bulk)
        );
        if (!table)
          return -1;
        table->entries = (Entry *)&table[1];
        *tablep = table;
      }
      table->entries[table->nentries] = (Entry){length, prefix, suffix};
      table->nentries++;
      if ((table->nentries & (table->nentries - 1)) == 0)
        return 1;
      return 0;
    }

    inline uint16_t get_key(GIF *gif, int key_size, uint8_t *sub_len, uint8_t *shift, uint8_t *byte) {
      int bits_read;
      int rpad;
      int frag_size;
      uint16_t key;

      key = 0;
      for (bits_read = 0; bits_read < key_size; bits_read += frag_size) {
        rpad = (*shift + bits_read) % 8;
        if (rpad == 0) {
          if (*sub_len == 0)
            fread(sub_len, 1, 1, gif->fp);
          fread(byte, 1, 1, gif->fp);
          (*sub_len)--;
        }
        frag_size = MIN(key_size - bits_read, 8 - rpad);
        key |= ((uint16_t)((*byte) >> rpad)) << bits_read;
      }

      key &= (1 << key_size) - 1;
      *shift = (*shift + key_size) % 8;
      return key;
    }

    inline int interlaced_line_index(int h, int y) {
      int p;

      p = (h - 1) / 8 + 1;
      if (y < p)
        return y * 8;
      y -= p;
      p = (h - 5) / 8 + 1;
      if (y < p)
        return y * 8 + 4;
      y -= p;
      p = (h - 3) / 4 + 1;
      if (y < p)
        return y * 4 + 2;
      y -= p;

      return y * 2 + 1;
    }

    inline int read_image_data(GIF *gif, int interlace) {
      uint8_t sub_len, shift, byte;
      int init_key_size, key_size, table_is_full;
      int frm_off, str_len, p, x, y;
      uint16_t key, clear, stop;
      int ret;
      Table *table;
      Entry entry = (Entry){0, 0, 0};
      off_t start, end;

      fread(&byte, 1, 1, gif->fp);
      key_size = (int)byte;
      start = ftell(gif->fp);
      discard_sub_blocks(gif);
      end = ftell(gif->fp);
      fseek(gif->fp, start, SEEK_SET);
      clear = 1 << key_size;
      stop = clear + 1;
      table = new_table(key_size);
      key_size++;
      init_key_size = key_size;
      sub_len = shift = 0;
      key = get_key(gif, key_size, &sub_len, &shift, &byte);
      frm_off = 0;
      ret = 0;
      table_is_full = 0;
      str_len = 0;
      while (1) {
        if (key == clear) {
          key_size = init_key_size;
          table->nentries = (1 << (key_size - 1)) + 2;
          table_is_full = 0;
        } else if (!table_is_full) {
          ret = add_entry(&table, str_len + 1, key, entry.suffix);
          if (ret == -1) {
            free(table);
            return -1;
          }
          if (table->nentries == 0x1000) {
            ret = 0;
            table_is_full = 1;
          }
        }
        key = get_key(gif, key_size, &sub_len, &shift, &byte);
        if (key == clear)
          continue;
        if (key == stop)
          break;
        if (ret == 1)
          key_size++;
        entry = table->entries[key];
        str_len = entry.length;
        while (1) {
          p = frm_off + entry.length - 1;
          x = p % gif->fw;
          y = p / gif->fw;
          if (interlace)
            y = interlaced_line_index((int)gif->fh, y);
          gif->frame[(gif->fy + y) * gif->width + gif->fx + x] = entry.suffix;
          if (entry.prefix == 0xFFF)
            break;
          else
            entry = table->entries[entry.prefix];
        }
        frm_off += str_len;
        if (key < table->nentries - 1 && !table_is_full)
          table->entries[table->nentries - 1].suffix = entry.suffix;
      }
      free(table);
      fread(&sub_len, 1, 1, gif->fp);
      fseek(gif->fp, end, SEEK_SET);
      return 0;
    }

    inline int read_image(GIF *gif) {
      uint8_t fisrz;
      int interlace;

      gif->fx = read_num(gif->fp);
      gif->fy = read_num(gif->fp);
      gif->fw = read_num(gif->fp);
      gif->fh = read_num(gif->fp);
      fread(&fisrz, 1, 1, gif->fp);
      interlace = fisrz & 0x40;

      if (fisrz & 0x80) {
        gif->lct.size = 1 << ((fisrz & 0x07) + 1);
        fread(gif->lct.colors, 1, 3 * gif->lct.size, gif->fp);
        gif->palette = &gif->lct;
      } else
        gif->palette = &gif->gct;

      return read_image_data(gif, interlace);
    }

    inline void render_frame_rect(GIF *gif, uint8_t *buffer) {
      int i, j, k;
      uint8_t index, *color;
      i = gif->fy * gif->width + gif->fx;
      for (j = 0; j < gif->fh; j++) {
        for (k = 0; k < gif->fw; k++) {
          index = gif->frame[(gif->fy + j) * gif->width + gif->fx + k];
          color = &gif->palette->colors[index * 3];
          if (!gif->gce.transparency || index != gif->gce.tindex) {
            int offset = (i + k) * BYTES_PER_PIXEL;
            buffer[offset + 0] = *(color + 0);
            buffer[offset + 1] = *(color + 1);
            buffer[offset + 2] = *(color + 2);
            if (index == gif->bgindex && gif->gce.transparency) {
              buffer[offset + 3] = 0;
            } else {
              buffer[offset + 3] = 255;
            }
          }
        }
        i += gif->width;
      }
    }

    inline void dispose(GIF *gif) {
      int i, j, k;
      uint8_t *bgcolor;
      switch (gif->gce.disposal) {
        case 2:
          bgcolor = &gif->palette->colors[gif->bgindex * 3];
          i = gif->fy * gif->width + gif->fx;
          for (j = 0; j < gif->fh; j++) {
            for (k = 0; k < gif->fw; k++) {
              memcpy(&gif->canvas[(i + k) * BYTES_PER_PIXEL], bgcolor, BYTES_PER_PIXEL);
            }
            i += gif->width;
          }
          break;
        case 3:
          break;
        default:
          render_frame_rect(gif, gif->canvas);
      }
    }

    inline int get_frame(GIF *gif) {
      char sep;

      dispose(gif);
      fread(&sep, 1, 1, gif->fp);
      while (sep != ',') {
        if (sep == ';')
          return 0;
        if (sep == '!')
          read_ext(gif);
        else
          return -1;
        fread(&sep, 1, 1, gif->fp);
      }
      if (read_image(gif) == -1)
        return -1;
      return 1;
    }

    inline void render_frame(GIF *gif, uint8_t *buffer) {
      memcpy(buffer, gif->canvas, gif->width * gif->height * BYTES_PER_PIXEL);
      render_frame_rect(gif, buffer);
    }

    inline void rewind(GIF *gif) {
      fseek(gif->fp, gif->anim_start, SEEK_SET);
    }

    inline void close_gif(GIF *gif) {
      fclose(gif->fp);
      free(gif);
    }
  } // namespace core

  class Gif {

  public:
    /**
     * Construct the Gif class
     * @param fname  the gif filename on the SD card (prefixed with /usd/)
     * @param parent the LVGL parent object
     */
    inline Gif(const char *fname, lv_obj_t *parent);

    /**
     * Destructs and cleans the Gif class
     */
    inline ~Gif();

    /**
     * Pauses the GIF task
     */
    inline void pause();

    /**
     * Resumes the GIF task
     */
    inline void resume();

    /**
     * Deletes GIF and frees all allocated memory
     */
    inline void clean();

  private:
    core::GIF *_gif = nullptr;
    void *_gifmem = nullptr;
    uint8_t *_buffer = nullptr;

    lv_color32_t *_cbuf = nullptr; // canvas buffer (ARGB8888)
    lv_obj_t *_canvas = nullptr;   // canvas object

    pros::task_t _task = nullptr; // render task

    /**
     * Cleans and frees all allocated memory
     */
    inline void _cleanup();

    /**
     * Render cycle, blocks until loop count exceeds gif loop count flag (if any)
     */
    inline void _render();

    /**
     * Calls _render()
     * @param arg Gif*
     */
    static void _render_task(void *arg);
  };

  inline Gif::Gif(const char *fname, lv_obj_t *parent) {
    FILE *fp = fopen(fname, "rb");

    if (fp != NULL) {
      fseek(fp, 0, SEEK_END);
      size_t len = ftell(fp);
      fseek(fp, 0, SEEK_SET);

      _gifmem = malloc(len);

      if (_gifmem != NULL) {
        fread(_gifmem, 1, len, fp);
      } else {
        std::cerr << "Gif::Gif - not enough memory for gif file" << std::endl;
      }
      fclose(fp);

      if (_gifmem != NULL) {
        FILE *memfp = fmemopen(_gifmem, len, "rb");

        _gif = core::open_gif(memfp);
        if (_gif == nullptr) {
          std::cerr << "Gif::Gif - error opening \"" + std::string(fname) + "\""
                    << std::endl;
          _cleanup();
          return;
        }

        _buffer = (uint8_t *)malloc(_gif->width * _gif->height * core::BYTES_PER_PIXEL);
        if (_buffer == nullptr) {
          _cleanup();
          std::cerr << "Gif::Gif - not enough memory for frame buffer"
                    << std::endl;
        } else {
          _cbuf = new lv_color32_t[_gif->width * _gif->height];
          _canvas = lv_canvas_create(parent);
          lv_canvas_set_buffer(_canvas, _cbuf, _gif->width, _gif->height, LV_COLOR_FORMAT_ARGB8888);
          _task = pros::c::task_create(
              _render_task, static_cast<void *>(this), TASK_PRIORITY_DEFAULT - 2, TASK_STACK_DEPTH_DEFAULT, ("GIF - \"" + std::string(fname) + "\"").c_str()
          );
        }
      }
    } else {
      std::cerr << "Gif::Gif - unable to open \"" + std::string(fname) +
                       "\" (file not found)"
                << std::endl;
    }
  }

  inline Gif::~Gif() {
    _cleanup();
  }

  inline void Gif::pause() {
    pros::c::task_suspend(_task);
  }

  inline void Gif::resume() {
    pros::c::task_resume(_task);
  }

  inline void Gif::clean() {
    _cleanup();
  }

  inline void Gif::_cleanup() {
    if (_canvas) {
      lv_obj_delete(_canvas);
      _canvas = nullptr;
    }
    if (_cbuf) {
      delete[] _cbuf;
      _cbuf = nullptr;
    }
    if (_buffer) {
      free(_buffer);
      _buffer = nullptr;
    }
    if (_gif) {
      core::close_gif(_gif);
      _gif = nullptr;
    }
    if (_gifmem) {
      free(_gifmem);
      _gifmem = nullptr;
    }
    if (_task) {
      pros::c::task_delete(_task);
      _task = nullptr;
    }
  }

  inline void Gif::_render() {

    for (size_t looped = 1;; looped++) {
      while (core::get_frame(_gif)) {
        int32_t now = pros::c::millis();

        core::render_frame(_gif, _buffer);

        for (size_t i = 0; i < _gif->height * _gif->width; i++) {
          uint8_t r = _buffer[(i * core::BYTES_PER_PIXEL)];
          uint8_t g = _buffer[(i * core::BYTES_PER_PIXEL) + 1];
          uint8_t b = _buffer[(i * core::BYTES_PER_PIXEL) + 2];
          uint8_t a = _buffer[(i * core::BYTES_PER_PIXEL) + 3];
          _cbuf[i] = lv_color32_make(r, g, b, a);
        }

        lv_obj_invalidate(_canvas);

        int32_t delay = _gif->gce.delay * 10;
        int32_t delta = pros::c::millis() - now;
        delay -= delta;

        if (delay > 0)
          pros::c::delay(delay);
      }

      if (looped == _gif->loop_count)
        break;
      core::rewind(_gif);
    }

    _cleanup();
  }

  inline void Gif::_render_task(void *arg) {
    Gif *instance = static_cast<Gif *>(arg);
    instance->_render();
  }
} // namespace lexlib::helpers::gif 