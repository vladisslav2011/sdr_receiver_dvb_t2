﻿/*
 *  Copyright 2020 Oleg Malyutin.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#ifndef P1_SYMBOL_H
#define P1_SYMBOL_H

#include <QObject>

#include "dvbt2_definition.h"
#include "DSP/fast_fourier_transform.h"
#include "DSP/buffers.hh"

#define P1_C_PART           542
#define P1_B_PART           482
#define P1_A_PART           1024
#define P1_LEN              2048
#define P1_FREQUENCY_SHIFT  1024
#define P1_CARRIER_SPASING  (SAMPLE_RATE / 1024.0f) //Hz
#define P1_ACTIVE_CARRIERS  384

class p1_symbol : public QObject
{
    Q_OBJECT

public:
    explicit p1_symbol(QObject *parent = nullptr);
    ~p1_symbol() override;

    bool execute(bool _gain_changed, float _level_detect,
                 const int _len_in, complex *_in, int &_consume,
                 complex* _buffer_sym, int &_idx_buffer_sym,
                 dvbt2_parameters &_dvbt2, double &_coarse_freq_offset,
                 bool &_p1_decoded, bool &_reset);

signals:
    void replace_spectrograph(const int _len_data, complex* _data);
    void replace_constelation(const int _len_data, complex* _data);
    void replace_oscilloscope(const int _len_data, complex* _data);
    void bad_signal();

private:
    complex fq_shift[P1_FREQUENCY_SHIFT];
    int idx_buffer = 0;
    delay_buffer<complex, P1_B_PART>        delay_b;
    delay_buffer<complex, P1_C_PART>        delay_c;
    delay_buffer<complex, P1_B_PART * 2>    delay_b_x2;
    delay_buffer<complex, 2>                delay_2;
    sum_of_buffer<complex, P1_B_PART>       average_b;
    sum_of_buffer<complex, P1_C_PART>       average_c;
    save_buffer<complex, P1_LEN>            p1_buffer;
    save_buffer<complex, P1_A_PART>         cor_buffer;
    float correlation = 0;

    /*const */float begin_threshold = 5.0e+5f;// 1.5e+5f FIX ME;
    /*const */float end_threshold = begin_threshold * 0.5f;

    float max_correlation = 0.0;
    bool correlation_detect = false;
    complex arg_max = {0.0, 0.0};
    complex* cor_os;
    int coarse_sample_offset = 0;
    bool p1_decoded = false;

    fast_fourier_transform* fft;
    complex* in_fft;
    complex* p1_fft;
    const int first_active_carrier = 86;
    int p1_randomize[P1_ACTIVE_CARRIERS];
    void init_p1_randomize();
    complex p1_dbpsk[P1_ACTIVE_CARRIERS];
    bool demodulate(complex *_p1, dvbt2_parameters &_dvbt2);
    void reset_buffer();

    const int p1_active_carriers[P1_ACTIVE_CARRIERS] =
    {
        44,  45,  47,  51,  54,  59,  62,  64,  65,  66,  70,  75,  78,  80,  81,  82,
        84,  85,  87,  88,  89,  90,  94,  96,  97,  98,  102, 107, 110, 112, 113, 114,
        116, 117, 119, 120, 121, 122, 124, 125, 127, 131, 132, 133, 135, 136, 137, 138,
        142, 144, 145, 146, 148, 149, 151, 152, 153, 154, 158, 160, 161, 162, 166, 171,
        172, 173, 175, 179, 182, 187, 190, 192, 193, 194, 198, 203, 206, 208, 209, 210,
        212, 213, 215, 216, 217, 218, 222, 224, 225, 226, 230, 235, 238, 240, 241, 242,
        244, 245, 247, 248, 249, 250, 252, 253, 255, 259, 260, 261, 263, 264, 265, 266,
        270, 272, 273, 274, 276, 277, 279, 280, 281, 282, 286, 288, 289, 290, 294, 299,
        300, 301, 303, 307, 310, 315, 318, 320, 321, 322, 326, 331, 334, 336, 337, 338,
        340, 341, 343, 344, 345, 346, 350, 352, 353, 354, 358, 363, 364, 365, 367, 371,
        374, 379, 382, 384, 385, 386, 390, 395, 396, 397, 399, 403, 406, 411, 412, 413,
        415, 419, 420, 421, 423, 424, 425, 426, 428, 429, 431, 435, 438, 443, 446, 448,
        449, 450, 454, 459, 462, 464, 465, 466, 468, 469, 471, 472, 473, 474, 478, 480,
        481, 482, 486, 491, 494, 496, 497, 498, 500, 501, 503, 504, 505, 506, 508, 509,
        511, 515, 516, 517, 519, 520, 521, 522, 526, 528, 529, 530, 532, 533, 535, 536,
        537, 538, 542, 544, 545, 546, 550, 555, 558, 560, 561, 562, 564, 565, 567, 568,
        569, 570, 572, 573, 575, 579, 580, 581, 583, 584, 585, 586, 588, 589, 591, 595,
        598, 603, 604, 605, 607, 611, 612, 613, 615, 616, 617, 618, 622, 624, 625, 626,
        628, 629, 631, 632, 633, 634, 636, 637, 639, 643, 644, 645, 647, 648, 649, 650,
        654, 656, 657, 658, 660, 661, 663, 664, 665, 666, 670, 672, 673, 674, 678, 683,
        684, 689, 692, 696, 698, 699, 701, 702, 703, 704, 706, 707, 708, 712, 714, 715,
        717, 718, 719, 720, 722, 723, 725, 726, 727, 729, 733, 734, 735, 736, 738, 739,
        740, 744, 746, 747, 748, 753, 756, 760, 762, 763, 765, 766, 767, 768, 770, 771,
        772, 776, 778, 779, 780, 785, 788, 792, 794, 795, 796, 801, 805, 806, 807, 809
    };
    const uint8_t s1_patterns[8][8] =
    {
        {0x12, 0x47, 0x21, 0x74, 0x1D, 0x48, 0x2E, 0x7B},
        {0x47, 0x12, 0x74, 0x21, 0x48, 0x1D, 0x7B, 0x2E},
        {0x21, 0x74, 0x12, 0x47, 0x2E, 0x7B, 0x1D, 0x48},
        {0x74, 0x21, 0x47, 0x12, 0x7B, 0x2E, 0x48, 0x1D},
        {0x1D, 0x48, 0x2E, 0x7B, 0x12, 0x47, 0x21, 0x74},
        {0x48, 0x1D, 0x7B, 0x2E, 0x47, 0x12, 0x74, 0x21},
        {0x2E, 0x7B, 0x1D, 0x48, 0x21, 0x74, 0x12, 0x47},
        {0x7B, 0x2E, 0x48, 0x1D, 0x74, 0x21, 0x47, 0x12}
    };
    const uint8_t s2_patterns[16][32] =
    {
        {0x12, 0x1D, 0x47, 0x48, 0x21, 0x2E, 0x74, 0x7B, 0x1D, 0x12, 0x48, 0x47, 0x2E, 0x21, 0x7B, 0x74,
         0x12, 0xE2, 0x47, 0xB7, 0x21, 0xD1, 0x74, 0x84, 0x1D, 0xED, 0x48, 0xB8, 0x2E, 0xDE, 0x7B, 0x8B},
        {0x47, 0x48, 0x12, 0x1D, 0x74, 0x7B, 0x21, 0x2E, 0x48, 0x47, 0x1D, 0x12, 0x7B, 0x74, 0x2E, 0x21,
         0x47, 0xB7, 0x12, 0xE2, 0x74, 0x84, 0x21, 0xD1, 0x48, 0xB8, 0x1D, 0xED, 0x7B, 0x8B, 0x2E, 0xDE},
        {0x21, 0x2E, 0x74, 0x7B, 0x12, 0x1D, 0x47, 0x48, 0x2E, 0x21, 0x7B, 0x74, 0x1D, 0x12, 0x48, 0x47,
         0x21, 0xD1, 0x74, 0x84, 0x12, 0xE2, 0x47, 0xB7, 0x2E, 0xDE, 0x7B, 0x8B, 0x1D, 0xED, 0x48, 0xB8},
        {0x74, 0x7B, 0x21, 0x2E, 0x47, 0x48, 0x12, 0x1D, 0x7B, 0x74, 0x2E, 0x21, 0x48, 0x47, 0x1D, 0x12,
         0x74, 0x84, 0x21, 0xD1, 0x47, 0xB7, 0x12, 0xE2, 0x7B, 0x8B, 0x2E, 0xDE, 0x48, 0xB8, 0x1D, 0xED},
        {0x1D, 0x12, 0x48, 0x47, 0x2E, 0x21, 0x7B, 0x74, 0x12, 0x1D, 0x47, 0x48, 0x21, 0x2E, 0x74, 0x7B,
         0x1D, 0xED, 0x48, 0xB8, 0x2E, 0xDE, 0x7B, 0x8B, 0x12, 0xE2, 0x47, 0xB7, 0x21, 0xD1, 0x74, 0x84},
        {0x48, 0x47, 0x1D, 0x12, 0x7B, 0x74, 0x2E, 0x21, 0x47, 0x48, 0x12, 0x1D, 0x74, 0x7B, 0x21, 0x2E,
         0x48, 0xB8, 0x1D, 0xED, 0x7B, 0x8B, 0x2E, 0xDE, 0x47, 0xB7, 0x12, 0xE2, 0x74, 0x84, 0x21, 0xD1},
        {0x2E, 0x21, 0x7B, 0x74, 0x1D, 0x12, 0x48, 0x47, 0x21, 0x2E, 0x74, 0x7B, 0x12, 0x1D, 0x47, 0x48,
         0x2E, 0xDE, 0x7B, 0x8B, 0x1D, 0xED, 0x48, 0xB8, 0x21, 0xD1, 0x74, 0x84, 0x12, 0xE2, 0x47, 0xB7},
        {0x7B, 0x74, 0x2E, 0x21, 0x48, 0x47, 0x1D, 0x12, 0x74, 0x7B, 0x21, 0x2E, 0x47, 0x48, 0x12, 0x1D,
         0x7B, 0x8B, 0x2E, 0xDE, 0x48, 0xB8, 0x1D, 0xED, 0x74, 0x84, 0x21, 0xD1, 0x47, 0xB7, 0x12, 0xE2},
        {0x12, 0xE2, 0x47, 0xB7, 0x21, 0xD1, 0x74, 0x84, 0x1D, 0xED, 0x48, 0xB8, 0x2E, 0xDE, 0x7B, 0x8B,
         0x12, 0x1D, 0x47, 0x48, 0x21, 0x2E, 0x74, 0x7B, 0x1D, 0x12, 0x48, 0x47, 0x2E, 0x21, 0x7B, 0x74},
        {0x47, 0xB7, 0x12, 0xE2, 0x74, 0x84, 0x21, 0xD1, 0x48, 0xB8, 0x1D, 0xED, 0x7B, 0x8B, 0x2E, 0xDE,
         0x47, 0x48, 0x12, 0x1D, 0x74, 0x7B, 0x21, 0x2E, 0x48, 0x47, 0x1D, 0x12, 0x7B, 0x74, 0x2E, 0x21},
        {0x21, 0xD1, 0x74, 0x84, 0x12, 0xE2, 0x47, 0xB7, 0x2E, 0xDE, 0x7B, 0x8B, 0x1D, 0xED, 0x48, 0xB8,
         0x21, 0x2E, 0x74, 0x7B, 0x12, 0x1D, 0x47, 0x48, 0x2E, 0x21, 0x7B, 0x74, 0x1D, 0x12, 0x48, 0x47},
        {0x74, 0x84, 0x21, 0xD1, 0x47, 0xB7, 0x12, 0xE2, 0x7B, 0x8B, 0x2E, 0xDE, 0x48, 0xB8, 0x1D, 0xED,
         0x74, 0x7B, 0x21, 0x2E, 0x47, 0x48, 0x12, 0x1D, 0x7B, 0x74, 0x2E, 0x21, 0x48, 0x47, 0x1D, 0x12},
        {0x1D, 0xED, 0x48, 0xB8, 0x2E, 0xDE, 0x7B, 0x8B, 0x12, 0xE2, 0x47, 0xB7, 0x21, 0xD1, 0x74, 0x84,
         0x1D, 0x12, 0x48, 0x47, 0x2E, 0x21, 0x7B, 0x74, 0x12, 0x1D, 0x47, 0x48, 0x21, 0x2E, 0x74, 0x7B},
        {0x48, 0xB8, 0x1D, 0xED, 0x7B, 0x8B, 0x2E, 0xDE, 0x47, 0xB7, 0x12, 0xE2, 0x74, 0x84, 0x21, 0xD1,
         0x48, 0x47, 0x1D, 0x12, 0x7B, 0x74, 0x2E, 0x21, 0x47, 0x48, 0x12, 0x1D, 0x74, 0x7B, 0x21, 0x2E},
        {0x2E, 0xDE, 0x7B, 0x8B, 0x1D, 0xED, 0x48, 0xB8, 0x21, 0xD1, 0x74, 0x84, 0x12, 0xE2, 0x47, 0xB7,
         0x2E, 0x21, 0x7B, 0x74, 0x1D, 0x12, 0x48, 0x47, 0x21, 0x2E, 0x74, 0x7B, 0x12, 0x1D, 0x47, 0x48},
        {0x7B, 0x8B, 0x2E, 0xDE, 0x48, 0xB8, 0x1D, 0xED, 0x74, 0x84, 0x21, 0xD1, 0x47, 0xB7, 0x12, 0xE2,
         0x7B, 0x74, 0x2E, 0x21, 0x48, 0x47, 0x1D, 0x12, 0x74, 0x7B, 0x21, 0x2E, 0x47, 0x48, 0x12, 0x1D}
    };
};

#endif // P1_SYMBOL_H
