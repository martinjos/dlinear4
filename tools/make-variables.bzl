def _var_providing_rule_impl(ctx):
  return [
      platform_common.TemplateVariableInfo({
          ctx.attr.var_name: ctx.attr.var_value,
      }),
  ]

var_providing_rule = rule(
    implementation = _var_providing_rule_impl,
    attrs = { "var_name": attr.string(),
              "var_value": attr.string() }
)

