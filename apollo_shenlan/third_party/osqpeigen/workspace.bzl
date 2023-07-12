"""Loads the rs_driver library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "osqpeigen",
        build_file = clean_dep("//third_party/osqpeigen:osqpeigen.BUILD"),
        path = "/apollo/third_party_shenlan/osqp-eigen/include/",
    )
