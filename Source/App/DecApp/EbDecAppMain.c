/*
* Copyright(c) 2019 Netflix, Inc.
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

/***************************************
 * Includes
 ***************************************/
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <inttypes.h>

#include "EbSvtAv1Dec.h"
#include "EbDecParamParser.h"
#include "EbMD5Utility.h"
#include "EbDecTime.h"

#ifdef _WIN32
#include <io.h>     /* _setmode() */
#include <fcntl.h>  /* _O_BINARY */
#endif

int init_pic_buffer(EbSvtIOFormat *pic_buffer, CLInput *cli) {
    switch (cli->fmt) {
    case EB_YUV400:
        pic_buffer->cb_stride = INT32_MAX;
        pic_buffer->cr_stride = INT32_MAX;
        pic_buffer->y_stride = cli->width;
        break;
    case EB_YUV420:
        pic_buffer->cb_stride = cli->width / 2;
        pic_buffer->cr_stride = cli->width / 2;
        pic_buffer->y_stride = cli->width;
        break;
    case EB_YUV422:
        pic_buffer->cb_stride = cli->width / 2;
        pic_buffer->cr_stride = cli->width / 2;
        pic_buffer->y_stride = cli->width;
        break;
    case EB_YUV444:
        pic_buffer->cb_stride = cli->width;
        pic_buffer->cr_stride = cli->width;
        pic_buffer->y_stride = cli->width;
        break;
    default:
        printf("Unsupported colour format. \n");
        return 0;
    }
    pic_buffer->width = cli->width;
    pic_buffer->height = cli->width;
    pic_buffer->luma_ext = NULL;
    pic_buffer->cb_ext = NULL;
    pic_buffer->cr_ext = NULL;
    pic_buffer->origin_x = 0;
    pic_buffer->origin_y = 0;
    return 0;
}

int read_input_frame(DecInputContext *input, uint8_t **buffer, size_t *bytes_read,
    size_t *buffer_size, int64_t *pts)
{
    CLInput *cli = input->cli_ctx;
    switch (cli->inFileType)
    {
    case FILE_TYPE_IVF:
        return read_ivf_frame(cli->inFile, buffer, bytes_read, buffer_size, pts);
        break;
    case FILE_TYPE_OBU:
        return obudec_read_temporal_unit(input, buffer, bytes_read,
            buffer_size);
        break;
    default:
        printf("Unsupported bitstream type. \n");
        return 0;
    }
}

void write_frame(EbBufferHeaderType *recon_buffer, CLInput *cli) {
    EbSvtIOFormat* img = (EbSvtIOFormat*)recon_buffer->p_buffer;

    const int bytes_per_sample = (cli->bit_depth == EB_EIGHT_BIT) ? 1 : 2;

    // Write luma plane
    unsigned char *buf = img->luma;
    int stride = img->y_stride;
    int w = img->width;
    int h = img->height;

    int y = 0;
    for (y = 0; y < h; ++y) {
        fwrite(buf, bytes_per_sample, w, cli->outFile);
        buf += (stride* bytes_per_sample);
    }
    if (img->color_fmt != EB_YUV400) {
        //Write chroma planes
        buf = img->cb;
        stride = img->cb_stride;
        if (img->color_fmt == EB_YUV420) {
            w /= 2;
            h /= 2;
        }
        else if (img->color_fmt == EB_YUV422) {
            w /= 2;
        }
        assert(img->color_fmt <= EB_YUV444);

        for (y = 0; y < h; ++y) {
            fwrite(buf, bytes_per_sample, w, cli->outFile);
            buf += (stride* bytes_per_sample);
        }

        buf = img->cr;
        stride = img->cr_stride;
        for (y = 0; y < h; ++y) {
            fwrite(buf, bytes_per_sample, w, cli->outFile);
            buf += (stride* bytes_per_sample);
        }
    }

    fflush(cli->outFile);
}

static void show_progress(int in_frame, uint64_t dx_time) {
    printf("\n%d frames decoded in %" PRId64 " us (%.2f fps)\r",
        in_frame, dx_time,
        (double)in_frame * 1000000.0 / (double)dx_time);
}

/***************************************
 * Decoder App Main
 ***************************************/
int32_t main(int32_t argc, char* argv[])
{
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    // GLOBAL VARIABLES
    EbErrorType            return_error = EB_ErrorNone; // Error Handling
    EbSvtAv1DecConfiguration  *config_ptr = (EbSvtAv1DecConfiguration*)
                                            malloc(sizeof(EbSvtAv1DecConfiguration));
    CLInput cli;
    cli.inFile = NULL;
    cli.outFile = NULL;
    cli.enable_md5 = 0;
    cli.fps_frm = 0;
    cli.fps_summary = 0;

    DecInputContext input = { NULL, NULL };
    ObuDecInputContext obu_ctx = { NULL, 0, 0, 0 };
    input.cli_ctx = &cli;
    input.obu_ctx = &obu_ctx;

    uint64_t stop_after = 0;
    uint32_t in_frame = 0;

    MD5Context md5_ctx;
    unsigned char md5_digest[16];

    struct EbDecTimer timer;
    uint64_t dx_time = 0;
    int fps_frm = 0;
    int fps_summary = 0;

    uint8_t *buf = NULL;
    size_t bytes_in_buffer = 0, buffer_size = 0;

    // Print Decoder Info
    printf("-------------------------------------\n");
    printf("SVT-AV1 Decoder\n");

    // Initialize config
    if (!config_ptr)
        return EB_ErrorInsufficientResources;
    EbComponentType* p_handle;
    void* p_app_data = NULL;

    return_error |= eb_dec_init_handle(&p_handle, p_app_data, config_ptr);
    if (return_error != EB_ErrorNone) goto fail;

    if (read_command_line(argc, argv, config_ptr, &cli, &obu_ctx) == 0 &&
        !eb_svt_dec_set_parameter(p_handle, config_ptr)) {
        return_error = eb_init_decoder(p_handle);
        if (return_error != EB_ErrorNone) {
            return_error |= eb_dec_deinit_handle(p_handle);
            goto fail;
        }

        assert(config_ptr->max_color_format <= EB_YUV444);
        assert(config_ptr->max_bit_depth <= EB_TWELVE_BIT);

        int enable_md5 = cli.enable_md5;

        fps_frm = cli.fps_frm;
        fps_summary = cli.fps_summary;

        EbBufferHeaderType *recon_buffer = NULL;
        recon_buffer = (EbBufferHeaderType*)malloc(sizeof(EbBufferHeaderType));
        recon_buffer->p_buffer = (uint8_t *)malloc(sizeof(EbSvtIOFormat));

        int size = (config_ptr->max_bit_depth == EB_EIGHT_BIT) ?
                                sizeof(uint8_t) : sizeof(uint16_t);
        size = size * cli.height * cli.width;

        ((EbSvtIOFormat *)recon_buffer->p_buffer)->luma = (uint8_t*)malloc(size);
        ((EbSvtIOFormat *)recon_buffer->p_buffer)->cb = (uint8_t*)malloc(size >> 2);
        ((EbSvtIOFormat *)recon_buffer->p_buffer)->cr = (uint8_t*)malloc(size >> 2);
        if (!init_pic_buffer((EbSvtIOFormat*)recon_buffer->p_buffer, &cli)) {
            printf("Decoding \n");
            EbAV1StreamInfo *stream_info = (EbAV1StreamInfo*)malloc(sizeof(EbAV1StreamInfo));
            EbAV1FrameInfo *frame_info = (EbAV1FrameInfo*)malloc(sizeof(EbAV1FrameInfo));

            if (config_ptr->skip_frames)
                fprintf(stderr, "Skipping first %" PRIu64 " frames.\n", config_ptr->skip_frames);
            uint64_t skip_frame = config_ptr->skip_frames;
            while (skip_frame) {
                if (!read_input_frame(&input, &buf, &bytes_in_buffer, &buffer_size, NULL)) break;
                skip_frame--;
            }
            stop_after = config_ptr->frames_to_be_decoded;
            if (enable_md5)
                md5_init(&md5_ctx);
            // Input Loop Thread
            while (read_input_frame(&input, &buf, &bytes_in_buffer, &buffer_size, NULL)) {
                if (!stop_after || in_frame < stop_after) {

                    dec_timer_start(&timer);

                    return_error |= eb_svt_decode_frame(p_handle, buf, bytes_in_buffer);

                    dec_timer_mark(&timer);
                    dx_time += dec_timer_elapsed(&timer);

                    in_frame++;

                    if (eb_svt_dec_get_picture(p_handle, recon_buffer, stream_info, frame_info) != EB_DecNoOutputPicture) {
                        if (fps_frm)
                            show_progress(in_frame , dx_time);

                        if (enable_md5)
                            write_md5(recon_buffer, &cli, &md5_ctx);
                        if(cli.outFile != NULL)
                            write_frame(recon_buffer, &cli);
                    }
                }
                else break;
            }
            if (fps_summary || fps_frm) {
                show_progress(in_frame, dx_time);
                printf("\n");
            }

            if (enable_md5) {
                md5_final(md5_digest, &md5_ctx);
                print_md5(md5_digest);
            }

            return_error |= eb_deinit_decoder(p_handle);

            free(frame_info);
            free(stream_info);
        }
        free(((EbSvtIOFormat *)recon_buffer->p_buffer)->cr);
        free(((EbSvtIOFormat *)recon_buffer->p_buffer)->cb);
        free(((EbSvtIOFormat *)recon_buffer->p_buffer)->luma);
        free(recon_buffer->p_buffer);
        free(recon_buffer);
        free(buf);
    }
    else
        printf("Error in configuration. \n");
    return_error |= eb_dec_deinit_handle(p_handle);

fail:
    if (cli.inFile) fclose(cli.inFile);
    if (cli.outFile) fclose(cli.outFile);

    free(config_ptr);

    return return_error;
}
