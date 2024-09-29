from lib.test.evaluation.environment import EnvSettings

def local_env_settings():
    settings = EnvSettings()

    # Set your local paths here.

    settings.davis_dir = ''
    settings.got10k_lmdb_path = ''
    settings.got10k_path = ''
    settings.got_packed_results_path = ''
    settings.got_reports_path = ''
    settings.lasot_extension_subset_path = ''
    settings.lasot_lmdb_path = ''
    settings.lasot_path = ''
    settings.network_path = '/home/mbzirc/tracker_ws/src/pytracking_image/SeqTrack/test/networks'    # Where tracking networks are stored.
    settings.nfs_path = '/home/mbzirc/tracker_ws/src/pytracking_image/SeqTrack/data/nfs'
    settings.otb_path = '/home/mbzirc/tracker_ws/src/pytracking_image/SeqTrack/data/OTB2015'
    settings.prj_dir = '/home/mbzirc/tracker_ws/src/pytracking_image/SeqTrack'
    settings.result_plot_path = '/home/mbzirc/tracker_ws/src/pytracking_image/SeqTrack/test/result_plots'
    settings.results_path = '/home/mbzirc/tracker_ws/src/pytracking_image/SeqTrack/test/tracking_results'    # Where to store tracking results
    settings.save_dir = '/home/mbzirc/tracker_ws/src/pytracking_image/SeqTrack'
    settings.segmentation_path = '/home/mbzirc/tracker_ws/src/pytracking_image/SeqTrack/test/segmentation_results'
    settings.tc128_path = '/home/mbzirc/tracker_ws/src/pytracking_image/SeqTrack/data/TC128'
    settings.tn_packed_results_path = ''
    settings.tnl2k_path = '/home/mbzirc/tracker_ws/src/pytracking_image/SeqTrack/data/tnl2k'
    settings.tpl_path = ''
    settings.trackingnet_path = '/home/mbzirc/tracker_ws/src/pytracking_image/SeqTrack/data/trackingnet'
    settings.uav_path = '/home/mbzirc/tracker_ws/src/pytracking_image/SeqTrack/data/UAV123'
    settings.vot_path = '/home/mbzirc/tracker_ws/src/pytracking_image/SeqTrack/data/VOT2019'
    settings.youtubevos_dir = ''

    return settings

