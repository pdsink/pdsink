import os
import shutil

Import("env")

out_dir = 'coverage'
env_name = env['PIOENV']
root = env.subst('$PROJECT_DIR')

def create_out_dir(target, source, env):
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir)

env.AddCustomTarget(
    name="coverage_report",
    dependencies=None,
    actions=[
        create_out_dir,
        f"platformio test -e {env_name}",
        f"lcov -d .pio/build/{env_name}/ -c -o {out_dir}/lcov.info",
        f"lcov --extract {out_dir}/lcov.info '{root}/src/*' '{root}/include/*' -o {out_dir}/filtered_lcov.info",
        f"genhtml -o {out_dir}/ --demangle-cpp {out_dir}/filtered_lcov.info"
    ],
    title="Coverage Report",
    description="Run tests and generate coverage report"
)
