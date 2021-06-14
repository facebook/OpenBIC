import setuptools

setuptools.setup(
        name="socsec",
        version="1.6",
        author="ASPEED Technology",
        author_email="bmc-sw@aspeedtech.com",
        description="Secure-boot utilities for ASPEED BMC SoCs",
        url="https://github.com/AspeedTech-BMC/socsec/",
        packages=setuptools.find_packages(),
        scripts=['tools/socsec', 'tools/otptool', 'tools/resocsec'],
        package_data={'socsec': ['otp_info/*']}
)
