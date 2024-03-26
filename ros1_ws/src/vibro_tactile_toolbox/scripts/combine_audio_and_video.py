import argparse
import moviepy.editor as mpe

def combine_video_with_audio(video_path, audio_path, output_path):

    video = mpe.VideoFileClip(video_path)
    video = video.set_audio(mpe.AudioFileClip(audio_path))
    video.write_videofile(output_path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Creates a combined video with audio.')
    parser.add_argument('--video_path', '-v', type=str, required=True,
                        help='video path')
    parser.add_argument('--audio_path', '-a', type=str, required=True,
                        help='audio path')
    parser.add_argument('--output_path', '-o', type=str, required=True,
                        help='output path')
    args = parser.parse_args()

    combine_video_with_audio(args.video_path, args.audio_path, args.output_path)