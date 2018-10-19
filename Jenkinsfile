#!/usr/bin/env groovy

pipeline {
  agent none
  stages {

    stage('Build') {
      steps {
        script {

          def build_nodes = [:]

          def docker_images = [
            armhf: "px4io/px4-dev-armhf:2019-07-29",
            base: "px4io/px4-dev-base-bionic:2019-07-29",
            nuttx: "px4io/px4-dev-nuttx:2019-07-29"
          ]


          def builds = [
            [
              board: "amber",
              configs: [
                "hello",
                ]
            ],
            [
              board: "arduino-due",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "arduino-mega2560",
              configs: [
                "hello",
                "nsh",
                ]
            ],
            [
              board: "avr32dev1",
              configs: [
                "nsh",
                "ostest",
                ]
            ],
            [
              board: "bambino-200e",
              configs: [
                "knsh",
                "max31855",
                "netnsh",
                "nsh",
                "usbnsh",
                ]
            ],
            [
              board: "beaglebone-black",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "b-l072z-lrwan1",
              configs: [
                "nsh",
                "sx127x",
                ]
            ],
            [
              board: "b-l475e-iot01a",
              configs: [
                "nsh",
                "spirit-6lowpan",
                "spirit-starhub",
                "spirit-starpoint",
                ]
            ],
            [
              board: "c5471evm",
              configs: [
                "httpd",
                "nettest",
                "nsh",
                ]
            ],
            [
              board: "clicker2-stm32",
              configs: [
                "knsh",
                "mrf24j40-6lowpan",
                "mrf24j40-mac",
                "mrf24j40-starhub",
                "mrf24j40-starpoint",
                "nsh",
                "usbnsh",
                "xbee-6lowpan",
                ]
            ],
            [
              board: "cloudctrl",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "demo9s12ne64",
              configs: [
                "ostest",
                ]
            ],
            [
              board: "dk-tm4c129x",
              configs: [
                "ipv6",
                "nsh",
                ]
            ],
            [
              board: "dummy",
              configs: [
                ]
            ],
            [
              board: "ea3131",
              configs: [
                "nsh",
                "pgnsh",
                "usbserial",
                ]
            ],
            [
              board: "ea3152",
              configs: [
                "ostest",
                ]
            ],
            [
              board: "eagle100",
              configs: [
                "httpd",
                "nettest",
                "nsh",
                "nxflat",
                "thttpd",
                ]
            ],
            [
              board: "efm32-g8xx-stk",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "efm32gg-stk3700",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "ekk-lm3s9b96",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "esp32-core",
              configs: [
                "nsh",
                "ostest",
                "smp",
                ]
            ],
            [
              board: "ez80f910200kitg",
              configs: [
                "ostest",
                ]
            ],
            [
              board: "ez80f910200zco",
              configs: [
                "dhcpd",
                "httpd",
                "nettest",
                "nsh",
                "poll",
                ]
            ],
            [
              board: "fire-stm32v2",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "flipnclick-pic32mz",
              configs: [
                "nsh",
                "nxlines",
                ]
            ],
            [
              board: "flipnclick-sam3x",
              configs: [
                "nsh",
                "nxlines",
                ]
            ],
            [
              board: "freedom-k28f",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "freedom-k64f",
              configs: [
                "netnsh",
                "nsh",
                ]
            ],
            [
              board: "freedom-k66f",
              configs: [
                "netnsh",
                "nsh",
                ]
            ],
            [
              board: "freedom-kl25z",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "freedom-kl26z",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "gapuino",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "hymini-stm32v",
              configs: [
                "nsh",
                "nsh2",
                "usbmsc",
                "usbnsh",
                "usbserial",
                ]
            ],
            [
              board: "imxrt1050-evk",
              configs: [
                "knsh",
                "libcxxtest",
                "netnsh",
                "nsh",
                ]
            ],
            [
              board: "imxrt1060-evk",
              configs: [
                "knsh",
                "libcxxtest",
                "netnsh",
                "nsh",
                ]
            ],
            [
              board: "kwikstik-k40",
              configs: [
                "ostest",
                ]
            ],
            [
              board: "launchxl-cc1310",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "launchxl-cc1312r1",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "launchxl-tms57004",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "lc823450-xgevk",
              configs: [
                "audio",
                "bt",
                "elf",
                "ipl2",
                "knsh",
                "kostest",
                "krndis",
                "nsh",
                "posix_spawn",
                "rndis",
                "usb",
                ]
            ],
            [
              board: "lincoln60",
              configs: [
                "netnsh",
                "nsh",
                "thttpd-binfs",
                ]
            ],
            [
              board: "lm3s6432-s2e",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "lm3s6965-ek",
              configs: [
                "discover",
                "nsh",
                "nx",
                "tcpecho",
                ]
            ],
            [
              board: "lm3s8962-ek",
              configs: [
                "nsh",
                "nx",
                ]
            ],
            [
              board: "lm4f120-launchpad",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "lpc4330-xplorer",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "lpc4337-ws",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "lpc4357-evb",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "lpc4370-link2",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "lpcxpresso-lpc1115",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "lpcxpresso-lpc1768",
              configs: [
                "dhcpd",
                "nsh",
                "nx",
                "thttpd",
                "usbmsc",
                ]
            ],
            [
              board: "lpcxpresso-lpc54628",
              configs: [
                "fb",
                "lvgl",
                "netnsh",
                "nsh",
                "nxwm",
                "pwfb",
                "pwlines",
                ]
            ],
            [
              board: "maple",
              configs: [
                "nsh",
                "nx",
                "usbnsh",
                ]
            ],
            [
              board: "max32660-evsys",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "mbed",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "mcb1700",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "mcu123-lpc214x",
              configs: [
                "composite",
                "nsh",
                "usbmsc",
                "usbserial",
                ]
            ],
            [
              board: "metro-m4",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "micropendous3",
              configs: [
                "hello",
                ]
            ],
            [
              board: "mikroe-stm32f4",
              configs: [
                "fulldemo",
                "kostest",
                "nsh",
                "nx",
                "nxlines",
                "nxtext",
                "usbnsh",
                ]
            ],
            [
              board: "mirtoo",
              configs: [
                "nsh",
                "nxffs",
                ]
            ],
            [
              board: "misoc",
              configs: [
                "hello",
                "nsh",
                ]
            ],
            [
              board: "moteino-mega",
              configs: [
                "hello",
                "nsh",
                ]
            ],
            [
              board: "moxa",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "ne64badge",
              configs: [
                "ostest",
                ]
            ],
            [
              board: "nr5m100-nexys4",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "nrf52-generic",
              configs: [
                "feather-nsh",
                "feather-userleds",
                "pca10040-nsh",
                "pca10040-wdog",
                ]
            ],
            [
              board: "ntosd-dm320",
              configs: [
                "nettest",
                "nsh",
                "poll",
                "udp",
                "webserver",
                ]
            ],
            [
              board: "nucleo-144",
              configs: [
                "f722-nsh",
                "f746-evalos",
                "f746-nsh",
                "f767-evalos",
                "f767-netnsh",
                "f767-nsh",
                ]
            ],
            [
              board: "nucleo-f072rb",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "nucleo-f091rc",
              configs: [
                "nsh",
                "sx127x",
                ]
            ],
            [
              board: "nucleo-f103rb",
              configs: [
                "adc",
                "nsh",
                "pwm",
                ]
            ],
            [
              board: "nucleo-f207zg",
              configs: [
                "adc",
                "nsh",
                "pwm",
                ]
            ],
            [
              board: "nucleo-f302r8",
              configs: [
                "highpri",
                "nsh",
                ]
            ],
            [
              board: "nucleo-f303re",
              configs: [
                "adc",
                "can",
                "hello",
                "nxlines",
                "pwm",
                "serialrx",
                ]
            ],
            [
              board: "nucleo-f303ze",
              configs: [
                "adc",
                "nsh",
                ]
            ],
            [
              board: "nucleo-f334r8",
              configs: [
                "adc",
                "highpri",
                "nsh",
                "spwm1",
                "spwm2",
                ]
            ],
            [
              board: "nucleo-f410rb",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "nucleo-f446re",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "nucleo-f4x1re",
              configs: [
                "f401-nsh",
                "f411-nsh",
                ]
            ],
            [
              board: "nucleo-h743zi",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "nucleo-l073rz",
              configs: [
                "nsh",
                "sx127x",
                ]
            ],
            [
              board: "nucleo-l152re",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "nucleo-l432kc",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "nucleo-l452re",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "nucleo-l476rg",
              configs: [
                "nsh",
                "nxdemo",
                ]
            ],
            [
              board: "nucleo-l496zg",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "nutiny-nuc120",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "olimex-efm32g880f128-stk",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "olimexino-stm32",
              configs: [
                "can",
                "composite",
                "nsh",
                "smallnsh",
                "tiny",
                ]
            ],
            [
              board: "olimex-lpc1766stk",
              configs: [
                "ftpc",
                "hidkbd",
                "hidmouse",
                "nettest",
                "nsh",
                "slip-httpd",
                "thttpd-binfs",
                "thttpd-nxflat",
                "usbmsc",
                "usbserial",
                "zmodem",
                ]
            ],
            [
              board: "olimex-lpc2378",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "olimex-lpc-h3131",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "olimex-stm32-e407",
              configs: [
                "discover",
                "netnsh",
                "nsh",
                "telnetd",
                "usbnsh",
                "webserver",
                ]
            ],
            [
              board: "olimex-stm32-h405",
              configs: [
                "usbnsh",
                ]
            ],
            [
              board: "olimex-stm32-h407",
              configs: [
                "nsh",
                "nsh_uext",
                ]
            ],
            [
              board: "olimex-stm32-p107",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "olimex-stm32-p207",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "olimex-stm32-p407",
              configs: [
                "dhtxx",
                "hidkbd",
                "kelf",
                "kmodule",
                "knsh",
                "module",
                "nsh",
                "zmodem",
                ]
            ],
            [
              board: "olimex-strp711",
              configs: [
                "nettest",
                "nsh",
                ]
            ],
            [
              board: "omnibusf4",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "open1788",
              configs: [
                "fb",
                "knsh",
                "knxterm",
                "nsh",
                "nxlines",
                "pdcurses",
                "pwfb",
                ]
            ],
            [
              board: "or1k",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "p112",
              configs: [
                "ostest",
                ]
            ],
            [
              board: "pcduino-a10",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "photon",
              configs: [
                "nsh",
                "rgbled",
                "usbnsh",
                "wlan",
                ]
            ],
            [
              board: "pic32mx7mmb",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "pic32mx-starterkit",
              configs: [
                "nsh",
                "nsh2",
                ]
            ],
            [
              board: "pic32mz-starterkit",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "pizero",
              configs: [
                ]
            ],
            [
              board: "qemu-i486",
              configs: [
                "nsh",
                "ostest",
                ]
            ],
            [
              board: "sabre-6quad",
              configs: [
                "nsh",
                "smp",
                ]
            ],
            [
              board: "sam3u-ek",
              configs: [
                "knsh",
                "nsh",
                "nx",
                "nxwm",
                ]
            ],
            [
              board: "sam4cmp-db",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "sam4e-ek",
              configs: [
                "nsh",
                "nxwm",
                "usbnsh",
                ]
            ],
            [
              board: "sam4l-xplained",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "sam4s-xplained",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "sam4s-xplained-pro",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "sama5d2-xult",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "sama5d3x-ek",
              configs: [
                "demo",
                "hello",
                "norboot",
                "nsh",
                "nx",
                "nxplayer",
                "nxwm",
                "ov2640",
                ]
            ],
            [
              board: "sama5d3-xplained",
              configs: [
                "bridge",
                "nsh",
                ]
            ],
            [
              board: "sama5d4-ek",
              configs: [
                "at25boot",
                "bridge",
                "dramboot",
                "elf",
                "ipv6",
                "knsh",
                "nsh",
                "nxwm",
                "ramtest",
                ]
            ],
            [
              board: "samd20-xplained",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "samd21-xplained",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "same70-xplained",
              configs: [
                "mrf24j40-starhub",
                "netnsh",
                "nsh",
                ]
            ],
            [
              board: "saml21-xplained",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "samv71-xult",
              configs: [
                "knsh",
                "module",
                "mrf24j40-starhub",
                "mxtxplnd",
                "netnsh",
                "nsh",
                "nxwm",
                "vnc",
                "vnxwm",
                ]
            ],
            [
              board: "shenzhou",
              configs: [
                "nsh",
                "nxwm",
                "thttpd",
                ]
            ],
            [
              board: "sim",
              configs: [
                "bas",
                "bluetooth",
                "configdata",
                "cxxtest",
                "dsptest",
                "fb",
                "ipforward",
                "loadable",
                "minibasic",
                "mount",
                "mtdpart",
                "mtdrwb",
                "nettest",
                "nsh",
                "nsh2",
                "nx",
                "nx11",
                "nxffs",
                "nxlines",
                "nxwm",
                "ostest",
                "pashello",
                "pf_ieee802154",
                "pktradio",
                "sixlowpan",
                "spiffs",
                "touchscreen",
                "traveler",
                "udgram",
                "unionfs",
                "userfs",
                "ustream",
                ]
            ],
            [
              board: "skp16c26",
              configs: [
                "ostest",
                ]
            ],
            [
              board: "stm3210e-eval",
              configs: [
                "composite",
                "nsh",
                "nsh2",
                "nx",
                "nxterm",
                "pm",
                "usbmsc",
                "usbserial",
                ]
            ],
            [
              board: "stm3220g-eval",
              configs: [
                "dhcpd",
                "nettest",
                "nsh",
                "nsh2",
                "nxwm",
                "telnetd",
                ]
            ],
            [
              board: "stm3240g-eval",
              configs: [
                "dhcpd",
                "discover",
                "fb",
                "knxwm",
                "nettest",
                "nsh",
                "nsh2",
                "nxterm",
                "nxwm",
                "telnetd",
                "webserver",
                "xmlrpc",
                ]
            ],
            [
              board: "stm32butterfly2",
              configs: [
                "nsh",
                "nshnet",
                "nshusbdev",
                "nshusbhost",
                ]
            ],
            [
              board: "stm32f051-discovery",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "stm32f072-discovery",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "stm32f103-minimum",
              configs: [
                "apds9960",
                "audio_tone",
                "buttons",
                "hello",
                "jlx12864g",
                "mcp2515",
                "nrf24",
                "nsh",
                "pwm",
                "rfid-rc522",
                "rgbled",
                "usbnsh",
                "userled",
                "veml6070",
                ]
            ],
            [
              board: "stm32f334-disco",
              configs: [
                "buckboost",
                "nsh",
                "powerled",
                ]
            ],
            [
              board: "stm32f3discovery",
              configs: [
                "nsh",
                "usbnsh",
                ]
            ],
            [
              board: "stm32f411e-disco",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "stm32f429i-disco",
              configs: [
                "adc",
                "extflash",
                "fb",
                "highpri",
                "lcd",
                "lvgl",
                "nsh",
                "nxhello",
                "nxwm",
                "usbmsc",
                "usbnsh",
                ]
            ],
            [
              board: "stm32f4discovery",
              configs: [
                "audio",
                "canard",
                "cxxtest",
                "elf",
                "hciuart",
                "ipv6",
                "kostest",
                "max31855",
                "max7219",
                "module",
                "netnsh",
                "nsh",
                "nxlines",
                "pm",
                "posix_spawn",
                "pseudoterm",
                "rgbled",
                "rndis",
                "testlibcxx",
                "usbmsc",
                "usbnsh",
                "winbuild",
                "xen1210",
                ]
            ],
            [
              board: "stm32f746g-disco",
              configs: [
                "fb",
                "lvgl",
                "netnsh",
                "nsh",
                "nxdemo",
                "nxterm",
                ]
            ],
            [
              board: "stm32f746-ws",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "stm32f769i-disco",
              configs: [
                "netnsh",
                "nsh",
                ]
            ],
            [
              board: "stm32l476-mdk",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "stm32l476vg-disco",
              configs: [
                "knsh",
                "nsh",
                ]
            ],
            [
              board: "stm32l4r9ai-disco",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "stm32ldiscovery",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "stm32_tiny",
              configs: [
                "nsh",
                "usbnsh",
                ]
            ],
            [
              board: "stm32vldiscovery",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "sure-pic32mx",
              configs: [
                "nsh",
                "usbnsh",
                ]
            ],
            [
              board: "teensy-2.0",
              configs: [
                "hello",
                "nsh",
                "usbmsc",
                ]
            ],
            [
              board: "teensy-3.x",
              configs: [
                "nsh",
                "usbnsh",
                ]
            ],
            [
              board: "teensy-lc",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "tm4c123g-launchpad",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "tm4c1294-launchpad",
              configs: [
                "ipv6",
                "nsh",
                ]
            ],
            [
              board: "tms570ls31x-usb-kit",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "twr-k60n512",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "twr-k64f120m",
              configs: [
                "netnsh",
                "nsh",
                ]
            ],
            [
              board: "u-blox-c027",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "ubw32",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "us7032evb1",
              configs: [
                "nsh",
                "ostest",
                ]
            ],
            [
              board: "viewtool-stm32f107",
              configs: [
                "ft80x",
                "highpri",
                "netnsh",
                "nsh",
                "tcpblaster",
                ]
            ],
            [
              board: "xmc4500-relax",
              configs: [
                "nsh",
                ]
            ],
            [
              board: "z16f2800100zcog",
              configs: [
                "nsh",
                "ostest",
                "pashello",
                ]
            ],
            [
              board: "z80sim",
              configs: [
                "nsh",
                "ostest",
                "pashello",
                ]
            ],
            [
              board: "z8encore000zco",
              configs: [
                "ostest",
                ]
            ],
            [
              board: "z8f64200100kit",
              configs: [
                "ostest",
                ]
            ],
            [
              board: "zkit-arm-1769",
              configs: [
                "hello",
                "nsh",
                "nxhello",
                "thttpd",
                ]
            ],
            [
              board: "zp214xpa",
              configs: [
                "nsh",
                "nxlines",
                ]
            ],
          ]


          for (def board_type = 0; board_type < builds.size(); board_type++) {

            build_nodes.put(
              builds[board_type].board,
              createBuildNode(docker_images.nuttx, builds[board_type].board, builds[board_type].configs)
            )

          }

        parallel build_nodes

        } // script
      } // steps
    } // stage Build

  } // stages
  environment {
    APPSDIR = 'NuttX-apps'
    CCACHE_DIR = '/tmp/ccache'
    CI = true
  }
  options {
    buildDiscarder(logRotator(numToKeepStr: '2', artifactDaysToKeepStr: '14'))
    timeout(time: 60, unit: 'MINUTES')
  }
}

def createBuildNode(String docker_image, String board, List<String> configs) {
  return {

    node {
      docker.withRegistry('https://registry.hub.docker.com', 'docker_hub_dagar') {
        docker.image(docker_image).inside('-e CCACHE_BASEDIR=${WORKSPACE} -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
          stage(board) {
            try {
              sh('export')
              checkout(scm)
              sh('make distclean')
              sh('git fetch --tags')
              sh('git clone --branch pr-cmake --depth 1 https://github.com/PX4/NuttX-apps.git')
              sh('ccache -z')

              for (def cfg = 0; cfg < configs.size(); cfg++) {
                sh('make ' + board + '/' + configs[cfg] || true)
              }

              sh('ccache -s')
              sh('make sizes')
              archiveArtifacts(allowEmptyArchive: true, artifacts: 'build/*/*.elf, build/*/*.bin', fingerprint: true, onlyIfSuccessful: true)
            }
            catch (exc) {
              throw (exc)
            }
            finally {
              sh('make distclean')
            }
          }
        }
      }
    }
  }
}

