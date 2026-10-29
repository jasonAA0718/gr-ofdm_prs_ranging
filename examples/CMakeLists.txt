install(FILES
    prs_ssrtt_initiator_multi_anchor.py
    prs_ssrtt_initiator_complete_epoch.py
    prs_ssrtt_responder_anchor0.py
    prs_ssrtt_responder_anchor1.py
    prs_ssrtt_initiator_multi_anchor.grc
    prs_ssrtt_initiator_complete_epoch.grc
    prs_ssrtt_responder_anchor0.grc
    prs_ssrtt_responder_anchor1.grc
    DESTINATION share/gnuradio/examples/ofdm_prs_ranging)

install(FILES
    "single link/prs_ssrtt_initiator.grc"
    "single link/prs_ssrtt_responder_newAcq.grc"
    DESTINATION "share/gnuradio/examples/ofdm_prs_ranging/single link")
