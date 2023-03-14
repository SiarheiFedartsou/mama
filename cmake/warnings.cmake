function(configure_warnings target)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
endfunction()