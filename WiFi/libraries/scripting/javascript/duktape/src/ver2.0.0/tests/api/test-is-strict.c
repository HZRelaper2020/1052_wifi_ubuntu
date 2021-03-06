#if defined(WICED)
#include "wiced_duktape_tests_api_internal.h"
#endif

/*===
outside: 1
inside: 1
===*/

static duk_ret_t my_func(duk_context *ctx) {
	printf("inside: %d\n", (int) duk_is_strict_call(ctx));
	return 0;
}

#if defined(WICED)
void wiced_duktape_tests_api_is_strict(duk_context *ctx) {
#else
void test(duk_context *ctx) {
#endif
	/* The context has no active function calls initially.  Default is to
	 * be strict, so duk_is_strict_call() returns 1.  It also returns 1
	 * whenever a Duktape/C function call is running, because all Duktape/C
	 * function calls are now strict.
	 */

	printf("outside: %d\n", (int) duk_is_strict_call(ctx));

	duk_push_c_function(ctx, my_func, 0);
	duk_call(ctx, 0);
}
