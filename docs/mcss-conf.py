DOXYFILE = "mcss-Doxyfile"
THEME_COLOR = "#cb4b16"
FAVICON = "https://3qzcxr28gq9vutx8scdn91zq-wpengine.netdna-ssl.com/wp-content/uploads/2016/05/cropped-EnviroDIY_LogoMaster_TrueSquare_V5_TwoTree_Trans_notext-192x192.png"
LINKS_NAVBAR1 = [
    (
        "About",
        "index",
        [
            ('<a href="page_calibration.html">Calibration</a>',),
            ('<a href="change_log.html">ChangeLog</a>',),
        ],
    ),
    (
        "The Main Class",
        "classgro_point",
        [],
    ),
    (
        "Examples",
        "page_the_examples",
        [
            ('<a href="example_get_values.html">Reading Sensor Values</a>',),
            (
                '<a href="example_change_modbus_settings.html">Changing Modbus Serial Settings</a>',
            ),
        ],
    ),
]
LINKS_NAVBAR2 = []
VERSION_LABELS = True
CLASS_INDEX_EXPAND_LEVELS = 2

STYLESHEETS = [
    "css/m-EnviroDIY+documentation.compiled.css",
]
